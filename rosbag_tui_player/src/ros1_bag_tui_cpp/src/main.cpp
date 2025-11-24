// main.cpp
//
// C++ Bag Player TUI (ROS1 + ncurses)
// - Keyboard behavior is aligned with roself.py BagControlUI:
//   Space      : play/pause
//   ← / →      : step backward / forward by step size (publish [t0, t1))
//   + / -      : step size ×2 / ÷2 (step_min / step_max same as roself)
//   0          : reset step size to 1s
//   Shift+←/→  : seek 10% of total duration (no publishing)
//   L          : loop on/off (no segment = loop whole bag; with segment = loop segment)
//   Shift+1..9 : set bookmark
//   1..9       : jump bookmark; in segment Modify mode: choose A/B bookmark
//   C / c      : segment state machine: Off→Modify; Modify/On then C -> Off + clear
//   R / r      : jump to start (segment start if exists, else 0.0)
//   q          : quit program
//
// Implemented:
//   - BagPlayer: publish original message type via topic_tools::ShapeShifter
//   - BagControlUI: full Segment + Loop + Bookmark logic (aligned with roself.py)
//   - LogPane + LogBuffer: log window (PgUp/PgDn/Home/End/F3/F4/o)
//
// TODO (can be extended later):
//   - Topic List / Topic View / Chart pages
//   - ROS2 version (BagPlayerROS2 + common interface)

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <topic_tools/shape_shifter.h>

#include <ncurses.h>
#include <fstream>
#include <string>
#include <map>
#include <deque>
#include <vector>
#include <array>
#include <optional>
#include <chrono>
#include <thread>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <locale.h>   // for setlocale

// ========================= LogBuffer & LogPane =========================

class LogBuffer {
public:
    explicit LogBuffer(size_t max_lines = 5000)
        : max_lines_(max_lines), scroll_(0), capture_enabled_(true) {}

    void append(const std::string& text) {
        if (!capture_enabled_) return;
        if (text.empty()) return;
        std::string line;
        for (char c : text) {
            if (c == '\n') {
                pushLine(line);
                line.clear();
            } else {
                line.push_back(c);
            }
        }
        if (!line.empty()) {
            pushLine(line);
        }
    }

    void clear() {
        lines_.clear();
        scroll_ = 0;
    }

    bool toggleCapture() {
        capture_enabled_ = !capture_enabled_;
        return capture_enabled_;
    }

    bool captureEnabled() const {
        return capture_enabled_;
    }

    // get visible lines within given height (scroll_ lines from bottom)
    std::vector<std::string> getView(int height) const {
        std::vector<std::string> out;
        if (height <= 0 || lines_.empty()) return out;

        int n = static_cast<int>(lines_.size());
        int start = std::max(0, n - height - scroll_);
        int end   = std::max(0, n - scroll_);
        if (start >= end) return out;
        out.reserve(end - start);
        for (int i = start; i < end; ++i) {
            out.push_back(lines_[i]);
        }
        return out;
    }

    void scrollUp(int n = 5) {
        scroll_ = std::min<int>(lines_.size(), scroll_ + n);
    }

    void scrollDown(int n = 5) {
        scroll_ = std::max(0, scroll_ - n);
    }

    void scrollHome() { // to very top
        scroll_ = static_cast<int>(lines_.size());
    }

    void scrollEnd() {  // to bottom (latest)
        scroll_ = 0;
    }

private:
    void pushLine(const std::string& ln) {
        if (lines_.size() >= max_lines_) {
            lines_.pop_front();
        }
        lines_.push_back(ln);
    }

    size_t max_lines_;
    std::deque<std::string> lines_;
    int  scroll_;
    bool capture_enabled_;
};

static LogBuffer g_log;

static void log_info(const std::string& msg) {
    g_log.append("[INFO] " + msg);
}

static void log_err(const std::string& msg) {
    g_log.append("[ERR] " + msg);
}

static void log_seg(const std::string& msg) {
    g_log.append("[SEG] " + msg);
}

static void log_bmk(const std::string& msg) {
    g_log.append("[BMK] " + msg);
}

struct Rect {
    int y{0}, x{0}, h{0}, w{0};
};

class LogPane {
public:
    LogPane()
        : at_bottom_(true),
          min_lines_(3),
          fixed_lines_(6),
          min_cols_(16),
          fixed_cols_(32) {}

    void toggleSide() { at_bottom_ = !at_bottom_; }

    void inc() {
        if (at_bottom_) {
            fixed_lines_ = std::min(20, fixed_lines_ + 1);
        } else {
            fixed_cols_ = std::min(80, fixed_cols_ + 2);
        }
    }

    void dec() {
        if (at_bottom_) {
            fixed_lines_ = std::max(min_lines_, fixed_lines_ - 1);
        } else {
            fixed_cols_ = std::max(min_cols_, fixed_cols_ - 2);
        }
    }

    void layout(int H, int W, Rect& mainRect, Rect& logRect) const {
        if (at_bottom_) {
            int log_h = std::min(std::max(min_lines_, fixed_lines_), std::max(1, H - 1));
            int main_h = std::max(1, H - log_h);
            mainRect = {0, 0, main_h, W};
            logRect  = {main_h, 0, log_h, W};
        } else {
            int log_w = std::min(std::max(min_cols_, fixed_cols_), std::max(1, W - 1));
            int main_w = std::max(1, W - log_w);
            mainRect = {0, 0, H, main_w};
            logRect  = {0, main_w, H, log_w};
        }
    }

    void draw(WINDOW* win) const {
        if (!win) return;
        int H, W;
        getmaxyx(win, H, W);
        werase(win);

        // title
        std::ostringstream oss;
        oss << " Logs [" << (at_bottom_ ? "BOTTOM" : "RIGHT")
            << "]  F2:side  F3:+  F4:-  PgUp/PgDn/Home/End  "
            << "o:capture=" << (g_log.captureEnabled() ? "on" : "off")
            << "  l:clear ";
        std::string title = oss.str();
        wattron(win, A_REVERSE);
        mvwaddnstr(win, 0, 0, title.c_str(), W - 1);
        wattroff(win, A_REVERSE);

        int view_h = std::max(1, H - 1);
        auto lines = g_log.getView(view_h);

        int row = 1;
        for (const auto& ln : lines) {
            if (row >= H) break;
            mvwaddnstr(win, row++, 0, ln.c_str(), W - 1);
        }

        wnoutrefresh(win);
    }

private:
    bool at_bottom_;
    int  min_lines_;
    int  fixed_lines_;
    int  min_cols_;
    int  fixed_cols_;
};

// ========================= BagPlayer (ROS1) =========================

class BagPlayer {
public:
    BagPlayer(const std::string& bag_path, ros::NodeHandle& nh)
        : nh_(nh) {
        bag_.open(bag_path, rosbag::bagmode::Read);

        rosbag::View view(bag_);
        if (view.begin() != view.end()) {
            t_start_  = view.getBeginTime().toSec();
            t_end_    = view.getEndTime().toSec();
            duration_ = std::max(0.0, t_end_ - t_start_);
        } else {
            t_start_ = t_end_ = 0.0;
            duration_ = 0.0;
        }
        duration_ = std::max(0.0, t_end_ - t_start_);
        cursor_   = 0.0;
        playing_  = false;
        rate_     = 1.0;

        // topic types & counts (similar to rosbag info)
        for (const auto& conn_info : view.getConnections()) {
            topic_types_[conn_info->topic] = conn_info->datatype;
            topic_counts_[conn_info->topic] = 0;
        }
        for (const auto& m : view) {
            topic_counts_[m.getTopic()]++;
        }
    }

    ~BagPlayer() {
        bag_.close();
    }

    double duration() const { return duration_; }
    double cursor()   const { return cursor_; }
    bool   playing()  const { return playing_; }
    double rate()     const { return rate_; }

    const std::map<std::string, std::string>& topicTypes() const {
        return topic_types_;
    }

    const std::map<std::string, size_t>& topicCounts() const {
        return topic_counts_;
    }

    void togglePlay() {
        playing_ = !playing_;
    }

    void play() {
        playing_ = true;
    }

    void pause() {
        playing_ = false;
    }

    void step(int direction, double sec) {
        if (duration_ <= 0.0) return;

        sec = std::max(0.0, sec);

        double old_cursor = cursor_;

        if (direction >= 0) {
            double rel0 = cursor_;
            double rel1 = std::min(duration_, cursor_ + sec);
            if (rel1 > rel0) {
                publishRange(rel0, rel1);
                cursor_ = rel1;
            }
        } else {
            double rel1 = cursor_;
            double rel0 = std::max(0.0, cursor_ - sec);
            if (rel1 > rel0) {
                publishRange(rel0, rel1);
                cursor_ = rel0;
            }
        }

        (void)old_cursor;
    }

    void setCursor(double rel) {
        cursor_ = std::max(0.0, std::min(duration_, rel));
    }

    // dt: wall-time delta
    void tick(double dt) {
        if (!playing_ || duration_ <= 0.0) return;
        if (dt <= 0.0) return;

        double rel0 = cursor_;
        double rel1 = std::min(duration_, cursor_ + dt * rate_);
        if (rel1 > rel0) {
            publishRange(rel0, rel1);
            cursor_ = rel1;
        }

        if (cursor_ >= duration_ - 1e-9) {
            pause();
        }
    }

    void setRate(double r) {
        rate_ = std::max(0.1, std::min(10.0, r));
    }

    double lastStepBegin() const { return last_step_begin_; }
    double lastStepEnd()   const { return last_step_end_;   }
    size_t lastStepCount() const { return last_step_count_; }

private:
    ros::Publisher& getPublisher(const std::string& topic,
                                 const std::string& md5,
                                 const std::string& datatype,
                                 const std::string& definition) {
        auto it = pubs_.find(topic);
        if (it != pubs_.end()) {
            return it->second;
        }
        ros::AdvertiseOptions opts(topic, 10, md5, datatype, definition);
        opts.latch = false;
        ros::Publisher pub = nh_.advertise(opts);
        auto res = pubs_.emplace(topic, pub);
        return res.first->second;
    }

    void publishRange(double rel0, double rel1) {
        last_step_begin_ = rel0;
        last_step_end_   = rel1;
        last_step_count_ = 0;

        ros::Time t0(t_start_ + rel0);
        ros::Time t1(t_start_ + rel1);

        rosbag::View view(bag_, t0, t1);

        for (const rosbag::MessageInstance& m : view) {
            const std::string& topic = m.getTopic();
            topic_tools::ShapeShifter::ConstPtr ss =
                m.instantiate<topic_tools::ShapeShifter>();
            if (!ss) continue;

            ros::Publisher& pub = getPublisher(
                topic,
                ss->getMD5Sum(),
                ss->getDataType(),
                ss->getMessageDefinition()
            );
            pub.publish(ss);
            ++last_step_count_;
        }
    }

    ros::NodeHandle nh_;
    rosbag::Bag bag_;

    double t_start_{0.0};
    double t_end_{0.0};
    double duration_{0.0};
    double cursor_{0.0};
    bool   playing_{false};
    double rate_{1.0};

    std::map<std::string, std::string> topic_types_;
    std::map<std::string, size_t>      topic_counts_;
    std::map<std::string, ros::Publisher> pubs_;

    double last_step_begin_{0.0};
    double last_step_end_{0.0};
    size_t last_step_count_{0};
};

// ========================= time formatting =========================

static std::string formatTime(double sec) {
    if (sec < 0) sec = 0;
    int isec = static_cast<int>(sec);
    int ms   = static_cast<int>((sec - isec) * 1000.0 + 0.5);

    int h = isec / 3600;
    int m = (isec % 3600) / 60;
    int s = isec % 60;

    std::ostringstream oss;
    oss << h << ":"
        << std::setw(2) << std::setfill('0') << m << ":"
        << std::setw(2) << std::setfill('0') << s << "."
        << std::setw(3) << std::setfill('0') << ms;
    return oss.str();
}

// ========================= BagControlUI =========================

class BagControlUI {
public:
    struct NoteItem {
        double rel_sec{0.0};        // 相对 bag 开始的时间（秒）
        std::string header;         // 原始头行 [time | +Xs]
        std::string text;           // 多行内容
    };

    BagControlUI(BagPlayer& player, const std::string& bag_path)
        : p_(player), bag_path_(bag_path) {
        step_ = 1.0;
        step_min_ = 0.1;
        step_max_ = std::max(1.0, p_.duration() * 0.10);
        loop_enabled_ = false;
        segment_arm_ = 0;
        segment_first_idx_ = -1;
        bookmarks_.fill(std::nullopt);
        // 加载与 bag 同名的 .txt 笔记
        loadNotesFromTxt();
    }

        // return true to quit program
    bool handleKey(int ch) {
        // if in Modify mode, q/ESC only exits Modify (segment unchanged)
        if (segment_arm_ != 0 && (ch == 'q' || ch == 'Q' || ch == 27)) {
            log_seg("Exit Modify mode (segment unchanged)");
            exitModify();
            return false;
        }

        // 上/下：切换笔记；Enter：跳转到当前选中笔记时间
        if (ch == KEY_UP) {
            if (!notes_.empty() && note_index_ > 0) {
                note_index_--;
            }
            return false;
        }
        if (ch == KEY_DOWN) {
            if (!notes_.empty() && note_index_ >= 0 &&
                note_index_ + 1 < static_cast<int>(notes_.size())) {
                note_index_++;
            }
            return false;
        }
        if (ch == '\n' || ch == '\r' || ch == KEY_ENTER) {
            if (!notes_.empty() && note_index_ >= 0 &&
                note_index_ < static_cast<int>(notes_.size())) {
                double t = notes_[note_index_].rel_sec;
                t = std::max(0.0, std::min(p_.duration(), t));
                p_.pause();
                p_.setCursor(t);
                std::ostringstream oss;
                oss << "Jump to note #" << (note_index_ + 1)
                    << " @ " << formatTime(t);
                log_info(oss.str());
            }
            return false;
        }

        // q/Q: quit program
        if (ch == 'q' || ch == 'Q') {
            p_.pause();
            return true;
        }

        // Shift+LEFT / Shift+RIGHT: seek 10% (no publish)
#ifdef KEY_SLEFT
        if (ch == KEY_SLEFT) {
            jumpPercent(-0.10);
            return false;
        }
#endif
#ifdef KEY_SRIGHT
        if (ch == KEY_SRIGHT) {
            jumpPercent(+0.10);
            return false;
        }
#endif

        // C/c: Off -> Modify; Modify/On -> Off + clear segment
        if (ch == 'c' || ch == 'C') {
            if (segment_arm_ != 0) {
                // Modify -> Off + clear
                exitModify();
                clearSegment();
                log_seg("Segment OFF: cleared, back to full range");
            } else if (loop_a_.has_value() && loop_b_.has_value()) {
                // On -> Off + clear
                clearSegment();
                log_seg("Segment OFF: cleared, back to full range");
            } else {
                // Off -> Modify
                enterModify();
            }
            return false;
        }

        // L: loop on/off
        if (ch == 'l' || ch == 'L') {
            loop_enabled_ = !loop_enabled_;
            if (loop_enabled_ && (!loop_a_.has_value() || !loop_b_.has_value())) {
                // no segment: loop whole bag (loop_a/b = None)
                loop_a_.reset();
                loop_b_.reset();
            }
            std::ostringstream oss;
            oss << "LOOP " << (loop_enabled_ ? "ON" : "OFF");
            log_info(oss.str());
            return false;
        }

        // R: jump to start (segment start if exists, otherwise 0)
        if (ch == 'r' || ch == 'R') {
            double target = 0.0;
            if (loop_a_.has_value() && loop_b_.has_value()) {
                double a = std::min(loop_a_.value(), loop_b_.value());
                target = a;
            }
            target = std::max(0.0, std::min(p_.duration(), target));
            p_.setCursor(target);
            std::ostringstream oss;
            oss << "Jump to start: " << formatTime(p_.cursor());
            log_info(oss.str());
            return false;
        }

        // play/step/step size & rate
        if (ch == ' ') {
            p_.togglePlay();
            return false;
        } else if (ch == KEY_LEFT) {
            p_.step(-1, step_);
            return false;
        } else if (ch == KEY_RIGHT) {
            p_.step(+1, step_);
            return false;
        }
        // '=': 放大步长；'-': 缩小步长
        else if (ch == '=') {
            step_ = std::min(step_max_, std::max(step_min_, step_ * 2.0));
            return false;
        } else if (ch == '-') {
            step_ = std::max(step_min_, step_ / 2.0);
            return false;
        }
        // '+': 提升播放速率 (Shift + '=')
        else if (ch == '+') {
            double r = p_.rate() * 2.0;
            p_.setRate(r);
            std::ostringstream oss;
            oss << "rate x2 -> " << p_.rate();
            log_info(oss.str());
            return false;
        }
        // '_': 降低播放速率 (Shift + '-')
        else if (ch == '_') {
            double r = p_.rate() / 2.0;
            p_.setRate(r);
            std::ostringstream oss;
            oss << "rate /2 -> " << p_.rate();
            log_info(oss.str());
            return false;
        }
        // '0': 重置步长为 1 秒
        else if (ch == '0') {
            step_ = 1.0;
            return false;
        }


        // digits: Shift+digit -> set bookmark; digit -> jump or segment modify input
        bool shifted = false;
        int digit = digitFromChar(ch, shifted);
        if (digit >= 1 && digit <= 9) {
            if (shifted) {
                handleBookmarkSet(digit);
            } else {
                if (segment_arm_ == 1 || segment_arm_ == 2) {
                    handleSegmentDigit(digit);
                } else {
                    handleBookmarkJump(digit);
                }
            }
            return false;
        }

        return false;
    }

    // per-frame tick: call BagPlayer::tick and handle loop/segment logic
    void tick(double dt) {
        p_.tick(dt);
        double cur = p_.cursor();

        bool has_segment = (loop_a_.has_value() && loop_b_.has_value());
        if (has_segment) {
            double a = std::min(loop_a_.value(), loop_b_.value());
            double b = std::max(loop_a_.value(), loop_b_.value());

            // clamp cursor into [a, b]
            if (cur < a) {
                p_.setCursor(a);
                cur = a;
            }
            if (cur > b) {
                p_.setCursor(b);
                cur = b;
            }

            // reached end of segment
            if (p_.playing() && cur >= b - 1e-9) {
                if (loop_enabled_) {
                    p_.setCursor(a);
                } else {
                    p_.pause();
                }
            }
        } else {
            // global loop
            if (loop_enabled_ && p_.playing() &&
                cur >= p_.duration() - 1e-9) {
                p_.setCursor(0.0);
            }
        }
    }

    void draw(WINDOW* win) {
        if (!win) return;
        int H, W;
        getmaxyx(win, H, W);
        werase(win);

        // title
        std::string state = p_.playing() ? "PLAY" : "PAUSE";
        std::ostringstream title;
        title << "BAG PLAYER (C++ / ROS1)  |  state=" << state
              << "  cursor=" << std::fixed << std::setprecision(2) << p_.cursor()
              << "s  step=" << step_ << "s"
              << "  rate=" << std::fixed << std::setprecision(2) << p_.rate() << "x";

        wattron(win, A_REVERSE);
        mvwaddnstr(win, 0, 0, title.str().c_str(), W - 1);
        wattroff(win, A_REVERSE);

        double cur = p_.cursor();
        double dur = std::max(1e-9, p_.duration());
        double ratio = std::min(1.0, std::max(0.0, cur / dur));

        std::string cur_s = formatTime(cur);
        std::string dur_s = formatTime(dur);

        int bar_y = 2;
        int bar_w = std::max(10, W - 2);
        int filled = static_cast<int>(ratio * bar_w + 0.5);
        if (filled < 0) filled = 0;
        if (filled > bar_w) filled = bar_w;

        std::string bar(bar_w, ' ');
        for (int i = 0; i < filled; ++i) bar[i] = '#';

        mvwaddnstr(win, bar_y, 0, bar.c_str(), W - 1);
        mvwaddnstr(win, bar_y + 1, 0, cur_s.c_str(),
                   std::min<int>(cur_s.size(), W - 1));
        mvwaddnstr(win, bar_y + 1,
                   std::max(0, W - 1 - (int)dur_s.size()),
                   dur_s.c_str(),
                   std::min<int>(dur_s.size(), W - 1));

        // last output stats
        double s0 = p_.lastStepBegin();
        double s1 = p_.lastStepEnd();
        size_t cnt = p_.lastStepCount();
        std::ostringstream out_info;
        out_info << "Last out: [" << std::fixed << std::setprecision(2)
                 << s0 << "s, " << s1 << "s)  msgs=" << cnt;
        mvwaddnstr(win, bar_y + 3, 0, out_info.str().c_str(), W - 1);

        // segment / loop / bookmark status
        std::string seg_txt = "-";
        if (loop_a_.has_value() && loop_b_.has_value()) {
            double a = std::min(loop_a_.value(), loop_b_.value());
            double b = std::max(loop_a_.value(), loop_b_.value());
            std::ostringstream ss;
            ss << formatTime(a) << " -> " << formatTime(b);
            seg_txt = ss.str();
        }
        std::string loop_txt = loop_enabled_ ? "ON" : "OFF";
        std::string seg_mode = segModeText();

        std::ostringstream seg_line;
        seg_line << "Loop: " << loop_txt
                 << "   Segment: " << seg_txt
                 << "   SegMode: " << seg_mode;
        mvwaddnstr(win, bar_y + 5, 0, seg_line.str().c_str(), W - 1);

        // bookmarks
        std::ostringstream bm;
        bm << "Bookmarks  ";
        for (int i = 1; i <= 9; ++i) {
            bm << i << ":";
            if (bookmarks_[i].has_value()) {
                bm << formatTime(bookmarks_[i].value());
            } else {
                bm << "--:--:--.---";
            }
            if (i != 9) bm << "  ";
        }
        mvwaddnstr(win, bar_y + 6, 0, bm.str().c_str(), W - 1);

        // Notes 显示区
        int note_row = bar_y + 8;
        if (notes_.empty()) {
            mvwaddnstr(win, note_row++, 0, "Notes: (none)", W - 1);
        } else {
            std::ostringstream ns;
            ns << "Notes: " << notes_.size();
            if (note_index_ >= 0) {
                ns << "  current #" << (note_index_ + 1)
                   << " @" << formatTime(notes_[note_index_].rel_sec);
            }
            mvwaddnstr(win, note_row++, 0, ns.str().c_str(), W - 1);

            if (note_index_ >= 0 && note_index_ < (int)notes_.size()) {
                const auto& n = notes_[note_index_];
                // 显示 header
                mvwaddnstr(win, note_row++, 0, n.header.c_str(), W - 1);
                // 显示内容第一行
                std::string first_line;
                std::istringstream iss(n.text);
                std::getline(iss, first_line);
                if (!first_line.empty()) {
                    mvwaddnstr(win, note_row++, 0, first_line.c_str(), W - 1);
                }
            }
        }

        // Bag info （往下挪几行）
        note_row += 1;
        mvwaddnstr(win, note_row++, 0, "Bag:", W - 1);
        mvwaddnstr(win, note_row++, 0, bag_path_.c_str(), W - 1);

        int row = note_row + 1;
        mvwaddnstr(win, row++, 0, "Topics:", W - 1);
        
        for (const auto& kv : p_.topicTypes()) {
            if (row >= H - 3) break;
            const std::string& topic = kv.first;
            const std::string& type  = kv.second;
            size_t count = 0;
            auto itc = p_.topicCounts().find(topic);
            if (itc != p_.topicCounts().end()) count = itc->second;

            std::ostringstream line;
            line << "  " << topic << "  [" << type << "]  msgs=" << count;
            mvwaddnstr(win, row++, 0, line.str().c_str(), W - 1);
        }

        // help text (ASCII only)
        std::string help1 =
            "SPACE: play/pause  LEFT/RIGHT: step  =/-: step x2/2  0: reset step  "
            "+/_: rate x2/2  Shift+LEFT/RIGHT: seek 10%  q: quit";
        std::string help2 =
            "L: loop  Shift+1..9: set bookmark  1..9: jump bookmark  "
            "C: segment mode Off<->Modify<->On  R: go to start  UP/DOWN: select note  Enter: jump note";


        wattron(win, A_REVERSE);
        mvwaddnstr(win, H - 2, 0, help1.c_str(), W - 1);
        mvwaddnstr(win, H - 1, 0, help2.c_str(), W - 1);
        wattroff(win, A_REVERSE);

        wnoutrefresh(win);
    }

private:

    // 从 header 行中解析 "+12.3s" 这样的相对时间
    double parseRelFromHeader(const std::string& header) const {
        size_t plus_pos = header.find('+');
        if (plus_pos == std::string::npos) return 0.0;
        size_t s_pos = header.find('s', plus_pos);
        if (s_pos == std::string::npos) return 0.0;
        std::string val = header.substr(plus_pos + 1, s_pos - plus_pos - 1);
        try {
            return std::stod(val);
        } catch (...) {
            return 0.0;
        }
    }

    void loadNotesFromTxt() {
        // bag_path -> .txt
        std::string txt_path = bag_path_;
        size_t dot = txt_path.rfind('.');
        if (dot != std::string::npos) {
            txt_path = txt_path.substr(0, dot) + ".txt";
        } else {
            txt_path += ".txt";
        }

        std::ifstream in(txt_path);
        if (!in.is_open()) {
            // 没有笔记文件，不报错，只是提示一下
            log_info("No note file found: " + txt_path);
            return;
        }

        log_info("Loading notes from: " + txt_path);

        notes_.clear();
        note_index_ = -1;

        std::string line;
        NoteItem current;
        bool in_note = false;

        while (std::getline(in, line)) {
            if (!line.empty() && line.front() == '[') {
                // 新的一条笔记
                if (in_note) {
                    notes_.push_back(current);
                }
                current = NoteItem{};
                current.header = line;
                current.text.clear();
                current.rel_sec = parseRelFromHeader(line);
                in_note = true;
            } else {
                if (in_note) {
                    if (!current.text.empty())
                        current.text.push_back('\n');
                    current.text += line;
                }
            }
        }
        if (in_note) {
            notes_.push_back(current);
        }

        if (!notes_.empty()) {
            note_index_ = 0;
            std::ostringstream oss;
            oss << "Loaded " << notes_.size() << " notes.";
            log_info(oss.str());
        } else {
            log_info("Note file exists but no notes parsed.");
        }
    }


    void jumpPercent(double frac) {
        if (p_.duration() <= 0.0) return;
        double jump = std::abs(frac) * p_.duration();
        double cur  = p_.cursor();
        double newpos = cur + (frac >= 0 ? jump : -jump);
        newpos = std::max(0.0, std::min(p_.duration(), newpos));
        p_.pause();
        p_.setCursor(newpos);
        // do not treat seek as a step (do not touch lastStep range/count)
        std::ostringstream oss;
        oss << "seek " << (frac >= 0 ? "+10%" : "-10%")
            << " -> " << formatTime(newpos);
        log_info(oss.str());
    }

    int digitFromChar(int ch, bool& shifted) const {
        shifted = false;
        if (ch >= '1' && ch <= '9') {
            return ch - '0';
        }
        char c = static_cast<char>(ch);
        const std::string symbols = "!@#$%^&*(";
        for (int i = 0; i < 9; ++i) {
            if (c == symbols[i]) {
                shifted = true;
                return i + 1;
            }
        }
        return 0;
    }

    void handleBookmarkSet(int digit) {
        bookmarks_[digit] = p_.cursor();
        std::ostringstream oss;
        oss << "Set bookmark " << digit << " at " << formatTime(p_.cursor());
        log_bmk(oss.str());
    }

    void handleBookmarkJump(int digit) {
        if (!bookmarks_[digit].has_value()) {
            std::ostringstream oss;
            oss << "Bookmark " << digit << " not set";
            log_bmk(oss.str());
            return;
        }
        double t = bookmarks_[digit].value();
        p_.setCursor(t);
        std::ostringstream oss;
        oss << "Jump to bookmark " << digit << " @ " << formatTime(t);
        log_bmk(oss.str());
    }

    void enterModify() {
        segment_arm_ = 1;
        segment_first_idx_ = -1;
        log_seg("Modify: press first digit (1-9) to choose start bookmark");
    }

    void exitModify() {
        segment_arm_ = 0;
        segment_first_idx_ = -1;
    }

    void clearSegment() {
        loop_a_.reset();
        loop_b_.reset();
    }

    void handleSegmentDigit(int digit) {
        if (segment_arm_ == 1) {
            if (!bookmarks_[digit].has_value()) {
                std::ostringstream oss;
                oss << "Bookmark " << digit
                    << " not set; exit Modify (segment unchanged)";
                log_seg(oss.str());
                exitModify();
                return;
            }
            segment_first_idx_ = digit;
            segment_arm_ = 2;
            log_seg("Modify: press second digit (1-9) to choose end bookmark");
        } else if (segment_arm_ == 2) {
            int idx1 = segment_first_idx_;
            int idx2 = digit;
            if (idx1 < 1 || idx1 > 9 ||
                !bookmarks_[idx1].has_value() ||
                !bookmarks_[idx2].has_value()) {
                log_seg("Bookmark not set; exit Modify (segment unchanged)");
                exitModify();
                return;
            }
            double t1 = bookmarks_[idx1].value();
            double t2 = bookmarks_[idx2].value();
            double a = std::min(t1, t2);
            double b = std::max(t1, t2);
            if (std::abs(a - b) < 1e-9) {
                log_seg("Start and end are the same; ignore and exit Modify");
                exitModify();
                return;
            }
            loop_a_ = a;
            loop_b_ = b;
            std::ostringstream oss;
            oss << "Segment ON: " << formatTime(a) << " -> " << formatTime(b)
                << " (" << (loop_enabled_ ? "LOOP" : "PAUSE at end") << ")";
            log_seg(oss.str());
            p_.setCursor(a);
            exitModify();
        }
    }

    std::string segModeText() const {
        if (segment_arm_ != 0) return "Modify";
        if (loop_a_.has_value() && loop_b_.has_value()) return "On";
        return "Off";
    }

    BagPlayer& p_;
    std::string bag_path_;

    double step_;
    double step_min_;
    double step_max_;

    bool loop_enabled_;
    std::optional<double> loop_a_;
    std::optional<double> loop_b_;
    std::array<std::optional<double>, 10> bookmarks_; // 1..9

    int segment_arm_;       // 0 = Off, 1 = waiting first digit, 2 = waiting second digit
    int segment_first_idx_; // 1..9

    // 笔记列表（从同名 .txt 读取）
    std::vector<NoteItem> notes_;
    int note_index_{-1};  // 当前选中的笔记索引，-1 表示没有
};

// ========================= main =========================

int main(int argc, char** argv) {
    if (argc < 2) {
        fprintf(stderr,
                "Usage: %s bag_file.bag\n"
                "  Keyboard behavior is similar to roself.py bag player.\n",
                argv[0]);
        return 1;
    }

    std::string bag_path = argv[1];

    ros::init(argc, argv, "ros1_bag_tui_player_cpp",
              ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    try {
        BagPlayer player(bag_path, nh);
        BagControlUI bag_ui(player, bag_path);
        LogPane log_pane;

        log_info("Opened bag: " + bag_path);
        {
            std::ostringstream oss;
            oss << "Duration=" << std::fixed << std::setprecision(2)
                << player.duration() << "s";
            log_info(oss.str());
        }

        // ncurses init
        setlocale(LC_ALL, "C");  // enforce plain C locale (ASCII-safe)
        initscr();
        cbreak();
        noecho();
        keypad(stdscr, TRUE);
        nodelay(stdscr, TRUE);
        curs_set(0);

        auto last_tp = std::chrono::steady_clock::now();
        bool running = true;

        while (ros::ok() && running) {
            int ch = getch();

            if (ch == KEY_RESIZE) {
                // terminal resized, will redraw next loop
            }

            // log pane hotkeys (global)
            if (ch != ERR) {
                if (ch == KEY_F(2)) {
                    log_pane.toggleSide();
                } else if (ch == KEY_F(3)) {
                    log_pane.inc();
                } else if (ch == KEY_F(4)) {
                    log_pane.dec();
                } else if (ch == KEY_PPAGE) {
                    g_log.scrollUp(10);
                } else if (ch == KEY_NPAGE) {
                    g_log.scrollDown(10);
                } else if (ch == KEY_HOME) {
                    g_log.scrollHome();
                } else if (ch == KEY_END) {
                    g_log.scrollEnd();
                } else if (ch == 'o' || ch == 'O') {
                    bool on = g_log.toggleCapture();
                    std::ostringstream oss;
                    oss << "output capture -> " << (on ? "on" : "off");
                    log_info(oss.str());
                } else if (ch == 'l' && false) {
                    // reserved: in case you want a key to clear logs
                }
            }

            // Bag control keys (forward to BagControlUI)
            if (ch != ERR) {
                if (bag_ui.handleKey(ch)) {
                    // user pressed q to quit
                    running = false;
                }
            }

            // tick
            auto now_tp = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(now_tp - last_tp).count();
            last_tp = now_tp;
            if (dt < 0) dt = 0;
            bag_ui.tick(dt);

            // draw
            erase();
            int H, W;
            getmaxyx(stdscr, H, W);

            Rect mainRect, logRect;
            log_pane.layout(H, W, mainRect, logRect);

            WINDOW* main_win = derwin(stdscr,
                                      mainRect.h, mainRect.w,
                                      mainRect.y, mainRect.x);
            WINDOW* log_win  = derwin(stdscr,
                                      logRect.h, logRect.w,
                                      logRect.y, logRect.x);

            bag_ui.draw(main_win);
            log_pane.draw(log_win);

            delwin(main_win);
            delwin(log_win);

            doupdate();

            ros::spinOnce();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        endwin();
    } catch (const std::exception& e) {
        fprintf(stderr, "Exception: %s\n", e.what());
        try {
            endwin();
        } catch (...) {}
        return 2;
    }

    return 0;
}
