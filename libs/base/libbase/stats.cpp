#include "stats.h"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

namespace stats {

namespace {

template <typename IntT> std::string formatInt(IntT v) {
    // Prints integral types as numbers (uint8_t as 0..255, not as char)
    char buf[64];
    auto *begin = buf;
    auto *end = buf + sizeof(buf);

    if constexpr (std::is_same_v<IntT, std::uint8_t>) {
        unsigned x = static_cast<unsigned>(v);
        auto res = std::to_chars(begin, end, x);
        return (res.ec == std::errc{}) ? std::string(begin, res.ptr) : std::string("0");
    } else if constexpr (std::is_signed_v<IntT>) {
        long long x = static_cast<long long>(v);
        auto res = std::to_chars(begin, end, x);
        return (res.ec == std::errc{}) ? std::string(begin, res.ptr) : std::string("0");
    } else {
        unsigned long long x = static_cast<unsigned long long>(v);
        auto res = std::to_chars(begin, end, x);
        return (res.ec == std::errc{}) ? std::string(begin, res.ptr) : std::string("0");
    }
}

inline std::string formatDoubleFixed(double x, int decimals) {
    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss << std::setprecision(decimals) << x;
    std::string s = oss.str();
    if (s == "-0.00" || s == "-0.0" || s == "-0")
        s = "0";
    return s;
}

inline std::string formatDoublePretty(double x, int max_decimals = 10) {
    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss << std::setprecision(max_decimals) << x;
    std::string s = oss.str();

    // Trim trailing zeros and dot
    while (!s.empty() && s.back() == '0')
        s.pop_back();
    if (!s.empty() && s.back() == '.')
        s.pop_back();

    if (s == "-0")
        s = "0";
    if (s.empty())
        s = "0";
    return s;
}

template <typename T> std::string formatPreviewValue(T v) {
    if constexpr (std::is_floating_point_v<T>) {
        return formatDoublePretty(static_cast<double>(v), 10);
    } else {
        return formatInt(v);
    }
}

template <typename T> std::vector<double> toDoubles(const std::vector<T> &values) {
    std::vector<double> v;
    v.reserve(values.size());
    for (const auto &x : values)
        v.push_back(static_cast<double>(x));
    return v;
}

} // namespace

template <AllowedType T> std::string toPercent(T part, T total) {
    int percent = std::round(part * 100.0 / total);
    return std::to_string(percent) + "%";
}

template <AllowedType T> T minValue(const std::vector<T> &values) {
    if (values.empty())
        throw std::invalid_argument("minValue: empty input");
    return *std::min_element(values.begin(), values.end());
}

template <AllowedType T> T maxValue(const std::vector<T> &values) {
    if (values.empty())
        throw std::invalid_argument("maxValue: empty input");
    return *std::max_element(values.begin(), values.end());
}

template <AllowedType T> double percentile(const std::vector<T> &values, double p) {
    if (values.empty())
        throw std::invalid_argument("percentile: empty input");
    if (!(p >= 0.0 && p <= 100.0))
        throw std::invalid_argument("percentile: p out of range [0,100]");

    const std::size_t n = values.size();
    if (n == 1)
        return static_cast<double>(values[0]);

    auto v = toDoubles(values);

    if (p <= 0.0)
        return *std::min_element(v.begin(), v.end());
    if (p >= 100.0)
        return *std::max_element(v.begin(), v.end());

    const double q = p / 100.0;
    const double pos = q * static_cast<double>(n - 1);
    const std::size_t i = static_cast<std::size_t>(std::floor(pos));
    const std::size_t j = static_cast<std::size_t>(std::ceil(pos));

    std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(i), v.end());
    const double a = v[i];
    if (j == i)
        return a;

    std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(j), v.end());
    const double b = v[j];

    const double t = pos - static_cast<double>(i);
    return a + t * (b - a);
}

template <AllowedType T> double sum(const std::vector<T> &values) {
    double total_sum = 0.0;
    for (const T &value: values) {
        total_sum += value;
    }
    return total_sum;
}

template <AllowedType T> double median(const std::vector<T> &values) { return percentile(values, 50.0); }

template <AllowedType T> std::string previewValues(const std::vector<T> &values) {
    const std::size_t n = values.size();

    std::string out;
    out.reserve(64);
    out += std::to_string(n);
    out += " values - [";

    if (n == 0) {
        out += "]";
        return out;
    }

    auto append = [&](const T &x) { out += formatPreviewValue(x); };

    if (n <= 10) {
        for (std::size_t i = 0; i < n; ++i) {
            if (i)
                out += ", ";
            append(values[i]);
        }
        out += "]";
        return out;
    }

    for (std::size_t i = 0; i < 5; ++i) {
        if (i)
            out += ", ";
        append(values[i]);
    }

    out += ", ... ";

    for (std::size_t i = n - 5; i < n; ++i) {
        if (i != n - 5)
            out += ", ";
        append(values[i]);
    }

    out += "]";
    return out;
}

template <AllowedType T>
    requires(!std::is_floating_point_v<T>)
std::string summaryStats(const std::vector<T> &values) {
    const std::size_t n = values.size();

    std::string out;
    out.reserve(128);
    out += std::to_string(n);
    out += " values - ";

    if (n == 0) {
        out += "(empty)";
        return out;
    }

    const T mnT = minValue(values);
    const T mxT = maxValue(values);

    const double p10 = percentile(values, 10.0);
    const double med = median(values);
    const double p90 = percentile(values, 90.0);

    out += "(min=" + formatInt(mnT);
    out += " 10%=" + formatDoublePretty(p10, 10);
    out += " median=" + formatDoublePretty(med, 10);
    out += " 90%=" + formatDoublePretty(p90, 10);
    out += " max=" + formatInt(mxT);
    out += ")";

    return out;
}

std::string summaryStats(const std::vector<float> &values, int decimals) {
    const std::size_t n = values.size();

    std::string out;
    out.reserve(128);
    out += std::to_string(n);
    out += " values - ";

    if (n == 0) {
        out += "(empty)";
        return out;
    }

    const float mn = minValue(values);
    const float mx = maxValue(values);

    const double p10 = percentile(values, 10.0);
    const double med = median(values);
    const double p90 = percentile(values, 90.0);

    out += "(min=" + formatDoubleFixed(static_cast<double>(mn), decimals);
    out += " 10%=" + formatDoubleFixed(p10, decimals);
    out += " median=" + formatDoubleFixed(med, decimals);
    out += " 90%=" + formatDoubleFixed(p90, decimals);
    out += " max=" + formatDoubleFixed(static_cast<double>(mx), decimals);
    out += ")";

    return out;
}

std::string summaryStats(const std::vector<double> &values, int decimals) {
    const std::size_t n = values.size();

    std::string out;
    out.reserve(128);
    out += std::to_string(n);
    out += " values - ";

    if (n == 0) {
        out += "(empty)";
        return out;
    }

    const double mn = minValue(values);
    const double mx = maxValue(values);

    const double p10 = percentile(values, 10.0);
    const double med = median(values);
    const double p90 = percentile(values, 90.0);

    out += "(min=" + formatDoubleFixed(mn, decimals);
    out += " 10%=" + formatDoubleFixed(p10, decimals);
    out += " median=" + formatDoubleFixed(med, decimals);
    out += " 90%=" + formatDoubleFixed(p90, decimals);
    out += " max=" + formatDoubleFixed(mx, decimals);
    out += ")";

    return out;
}

// ---- Explicit instantiations (only allowed types) ----
template std::string toPercent<int>(int part, int total);
template std::string toPercent<float>(float part, float total);
template std::string toPercent<double>(double part, double total);
template std::string toPercent<std::size_t>(std::size_t part, std::size_t total);
template std::string toPercent<std::uint8_t>(std::uint8_t part, std::uint8_t total);

template int minValue<int>(const std::vector<int> &);
template float minValue<float>(const std::vector<float> &);
template double minValue<double>(const std::vector<double> &);
template std::size_t minValue<std::size_t>(const std::vector<std::size_t> &);
template std::uint8_t minValue<std::uint8_t>(const std::vector<std::uint8_t> &);

template int maxValue<int>(const std::vector<int> &);
template float maxValue<float>(const std::vector<float> &);
template double maxValue<double>(const std::vector<double> &);
template std::size_t maxValue<std::size_t>(const std::vector<std::size_t> &);
template std::uint8_t maxValue<std::uint8_t>(const std::vector<std::uint8_t> &);

template double sum<int>(const std::vector<int> &);
template double sum<float>(const std::vector<float> &);
template double sum<double>(const std::vector<double> &);
template double sum<std::size_t>(const std::vector<std::size_t> &);
template double sum<std::uint8_t>(const std::vector<std::uint8_t> &);

template double median<int>(const std::vector<int> &);
template double median<float>(const std::vector<float> &);
template double median<double>(const std::vector<double> &);
template double median<std::size_t>(const std::vector<std::size_t> &);
template double median<std::uint8_t>(const std::vector<std::uint8_t> &);

template double percentile<int>(const std::vector<int> &, double);
template double percentile<float>(const std::vector<float> &, double);
template double percentile<double>(const std::vector<double> &, double);
template double percentile<std::size_t>(const std::vector<std::size_t> &, double);
template double percentile<std::uint8_t>(const std::vector<std::uint8_t> &, double);

template std::string previewValues<int>(const std::vector<int> &);
template std::string previewValues<float>(const std::vector<float> &);
template std::string previewValues<double>(const std::vector<double> &);
template std::string previewValues<std::size_t>(const std::vector<std::size_t> &);
template std::string previewValues<std::uint8_t>(const std::vector<std::uint8_t> &);

template std::string summaryStats<int>(const std::vector<int> &);
template std::string summaryStats<std::size_t>(const std::vector<std::size_t> &);
template std::string summaryStats<std::uint8_t>(const std::vector<std::uint8_t> &);

} // namespace stats