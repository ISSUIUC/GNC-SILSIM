#include "magnetic_model.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace magnetic_model {
namespace {

constexpr double kEarthReferenceRadiusKm = 6371.2;
constexpr double kPi = 3.14159265358979323846;
constexpr double kPoleEpsilonDeg = 1.0e-8;

struct SchmidtCoefficients {
    int max_degree = 0;
    std::vector<std::vector<double>> g;
    std::vector<std::vector<double>> h;
    std::vector<std::vector<double>> g_sv;
    std::vector<std::vector<double>> h_sv;
};

struct NormalizedCoefficient {
    int n = 0;
    int m = 0;
    double base = 0.0;
    double secular_variation = 0.0;
};

struct NormalizedModel {
    int max_degree = 0;
    std::vector<NormalizedCoefficient> g;
    std::vector<NormalizedCoefficient> h;
};

double deg_to_rad(double degrees) {
    return degrees * kPi / 180.0;
}

std::string trim(const std::string& text) {
    const auto begin = text.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return "";
    }

    const auto end = text.find_last_not_of(" \t\r\n");
    return text.substr(begin, end - begin + 1);
}

std::vector<std::string> split_csv_line(const std::string& line) {
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) {
        fields.push_back(trim(field));
    }
    return fields;
}

std::string resolve_coeff_path(const std::string& requested_path) {
    namespace fs = std::filesystem;

    if (!requested_path.empty()) {
        fs::path explicit_path(requested_path);
        if (fs::exists(explicit_path)) {
            return explicit_path.string();
        }
        throw std::runtime_error("IGRF coefficient file not found: " + requested_path);
    }

    const std::vector<fs::path> candidates = {
        fs::current_path() / "../LookUp/IGRF13coeffs.csv",
        fs::current_path() / "../../LookUp/IGRF13coeffs.csv",
        fs::current_path() / "LookUp/IGRF13coeffs.csv"
    };

    for (const auto& candidate : candidates) {
        if (fs::exists(candidate)) {
            return fs::canonical(candidate).string();
        }
    }

    throw std::runtime_error("Unable to locate IGRF13coeffs.csv relative to the current working directory");
}

SchmidtCoefficients load_raw_coefficients(const std::string& coeff_path) {
    std::ifstream file(coeff_path);
    if (!file) {
        throw std::runtime_error("Failed to open magnetic coefficient file: " + coeff_path);
    }

    SchmidtCoefficients coeffs;
    std::string line;

    if (!std::getline(file, line)) {
        throw std::runtime_error("Magnetic coefficient file is empty: " + coeff_path);
    }

    struct Row {
        char type = 'g';
        int n = 0;
        int m = 0;
        double igrf = 0.0;
        double sv = 0.0;
    };

    std::vector<Row> rows;
    while (std::getline(file, line)) {
        if (trim(line).empty()) {
            continue;
        }

        const auto fields = split_csv_line(line);
        if (fields.size() < 5) {
            throw std::runtime_error("Malformed coefficient row in: " + coeff_path);
        }

        Row row;
        row.type = fields[0].empty() ? 'g' : fields[0][0];
        row.n = std::stoi(fields[1]);
        row.m = std::stoi(fields[2]);
        row.igrf = std::stod(fields[3]);
        row.sv = std::stod(fields[4]);
        coeffs.max_degree = std::max(coeffs.max_degree, row.n);
        rows.push_back(row);
    }

    const int size = coeffs.max_degree + 1;
    coeffs.g.assign(size, std::vector<double>(size, 0.0));
    coeffs.h.assign(size, std::vector<double>(size, 0.0));
    coeffs.g_sv.assign(size, std::vector<double>(size, 0.0));
    coeffs.h_sv.assign(size, std::vector<double>(size, 0.0));

    for (const auto& row : rows) {
        if (row.type == 'g' || row.type == 'G') {
            coeffs.g[row.n][row.m] = row.igrf;
            coeffs.g_sv[row.n][row.m] = row.sv;
        } else {
            coeffs.h[row.n][row.m] = row.igrf;
            coeffs.h_sv[row.n][row.m] = row.sv;
        }
    }

    return coeffs;
}

NormalizedModel quasi_normalize(const std::string& coeff_path) {
    const SchmidtCoefficients raw = load_raw_coefficients(coeff_path);
    const int max_degree = raw.max_degree;

    std::vector<std::vector<double>> schmidt(max_degree + 1, std::vector<double>(max_degree + 1, 0.0));
    std::vector<NormalizedCoefficient> g_norm;
    std::vector<NormalizedCoefficient> h_norm;
    g_norm.reserve((max_degree + 1) * (max_degree + 2) / 2);
    h_norm.reserve((max_degree + 1) * (max_degree + 2) / 2);

    for (int n = 1; n <= max_degree; ++n) {
        for (int m = 0; m <= n; ++m) {
            if (m > 1) {
                schmidt[n][m] = schmidt[n][m - 1] * std::sqrt(static_cast<double>(n - m + 1) / static_cast<double>(n + m));
            } else if (m > 0) {
                schmidt[n][m] = schmidt[n][m - 1] * std::sqrt(2.0 * static_cast<double>(n - m + 1) / static_cast<double>(n + m));
            } else if (n == 1) {
                schmidt[n][0] = 1.0;
            } else {
                schmidt[n][0] = schmidt[n - 1][0] * static_cast<double>(2 * n - 1) / static_cast<double>(n);
            }

            g_norm.push_back({n, m, raw.g[n][m] * schmidt[n][m], raw.g_sv[n][m] * schmidt[n][m]});
            h_norm.push_back({n, m, raw.h[n][m] * schmidt[n][m], raw.h_sv[n][m] * schmidt[n][m]});
        }
    }

    return {max_degree, std::move(g_norm), std::move(h_norm)};
}

const NormalizedModel& get_cached_model(const std::string& coeff_path) {
    static std::unordered_map<std::string, NormalizedModel> cache;

    auto it = cache.find(coeff_path);
    if (it == cache.end()) {
        it = cache.emplace(coeff_path, quasi_normalize(coeff_path)).first;
    }
    return it->second;
}

Vec3 sphere_to_cartesian(double br, double bt, double bp) {
    const double e = 0.0;
    const double bx = -bt * std::cos(e) - br * std::sin(e);
    const double by = bp;
    const double bz = bt * std::sin(e) - br * std::cos(e);
    return {bx, by, bz};
}

} // namespace

Vec3 magnet(double r_km,
            double latitude_deg,
            double longitude_deg,
            double days_since_2020,
            const std::string& coeff_path) {
    if (r_km <= 0.0) {
        throw std::invalid_argument("Geocentric radius must be positive");
    }

    double latitude = latitude_deg;
    if (std::abs(latitude) < kPoleEpsilonDeg) {
        latitude = kPoleEpsilonDeg;
    } else if (std::abs(180.0 - latitude) < kPoleEpsilonDeg) {
        latitude = 180.0 - kPoleEpsilonDeg;
    }

    const double theta = deg_to_rad(90.0 - latitude);
    const double phi = deg_to_rad(longitude_deg);
    const double sin_theta = std::sin(theta);
    if (std::abs(sin_theta) < 1.0e-12) {
        throw std::runtime_error("Magnetic model encountered a polar singularity");
    }

    const std::string resolved_path = resolve_coeff_path(coeff_path);
    const NormalizedModel& model = get_cached_model(resolved_path);

    std::vector<std::vector<double>> g(model.max_degree + 1, std::vector<double>(model.max_degree + 1, 0.0));
    std::vector<std::vector<double>> h(model.max_degree + 1, std::vector<double>(model.max_degree + 1, 0.0));

    for (std::size_t i = 0; i < model.g.size(); ++i) {
        const auto& g_coeff = model.g[i];
        const auto& h_coeff = model.h[i];
        const double scale = days_since_2020 / 365.0;
        g[g_coeff.n][g_coeff.m] = g_coeff.base + g_coeff.secular_variation * scale;
        h[h_coeff.n][h_coeff.m] = h_coeff.base + h_coeff.secular_variation * scale;
    }

    double br = 0.0;
    double bt = 0.0;
    double bp = 0.0;

    for (int m = 0; m <= model.max_degree; ++m) {
        double p11 = 1.0;
        double p10 = 1.0;
        double dp11 = 0.0;
        double dp10 = 0.0;
        double p20 = 0.0;
        double dp20 = 0.0;

        for (int n = 1; n <= model.max_degree; ++n) {
            if (m > n) {
                continue;
            }

            double p_curr = 0.0;
            double dp_curr = 0.0;
            if (n == m) {
                p_curr = p11 * sin_theta;
                dp_curr = dp11 * sin_theta + p11 * std::cos(theta);
                p11 = p_curr;
                dp11 = dp_curr;
                p10 = p11;
                dp10 = dp11;
                p20 = 0.0;
                dp20 = 0.0;
            } else if (n == 1) {
                p_curr = std::cos(theta) * p10;
                dp_curr = dp10 * std::cos(theta) - sin_theta * p10;
                p20 = p10;
                dp20 = dp10;
                p10 = p_curr;
                dp10 = dp_curr;
            } else {
                const double numerator = static_cast<double>((n - 1) * (n - 1) - m * m);
                const double denominator = static_cast<double>((2 * n - 1) * (2 * n - 3));
                const double k = numerator / denominator;
                p_curr = p10 * std::cos(theta) - k * p20;
                dp_curr = dp10 * std::cos(theta) - p10 * sin_theta - k * dp20;
                p20 = p10;
                dp20 = dp10;
                p10 = p_curr;
                dp10 = dp_curr;
            }

            const double common = std::pow(kEarthReferenceRadiusKm / r_km, n + 2);
            const double cos_m_phi = std::cos(static_cast<double>(m) * phi);
            const double sin_m_phi = std::sin(static_cast<double>(m) * phi);
            const double gh_term = g[n][m] * cos_m_phi + h[n][m] * sin_m_phi;

            br += common * (n + 1) * gh_term * p_curr;
            bt += common * gh_term * dp_curr;
            bp += common * m * (-g[n][m] * sin_m_phi + h[n][m] * cos_m_phi) * p_curr;
        }
    }

    return sphere_to_cartesian(br, -bt, -bp / sin_theta);
}

} 
