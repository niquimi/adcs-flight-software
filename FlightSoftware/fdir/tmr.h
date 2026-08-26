#pragma once

template <typename T>
class Tmr {
public:
    Tmr() = default;
    explicit Tmr(const T& v) { set(v); }

    void set(const T& v) { a_ = b_ = c_ = v; }

    T& replica(int i) {
        return (i == 1) ? b_ : (i == 2) ? c_ : a_;
    }

    bool vote(T& out) const {
        if (a_ == b_ || a_ == c_) {
            out = a_;
            return !(a_ == b_ && b_ == c_);
        }
        if (b_ == c_) {
            out = b_;
            return true;
        }
        out = a_;
        return true;
    }

    bool readAndRepair(T& out) {
        const bool mismatch = vote(out);
        if (mismatch && hasMajority()) {
            set(out);
        }
        return mismatch;
    }

    bool hasMajority() const {
        return a_ == b_ || a_ == c_ || b_ == c_;
    }

private:
    T a_{};
    T b_{};
    T c_{};
};
