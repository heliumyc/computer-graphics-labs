////////////////////////////////////////////////////////////////////////////////
#include <algorithm>
#include <complex>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>
#include <cmath>
////////////////////////////////////////////////////////////////////////////////

typedef std::complex<double> Point;
typedef std::vector<Point> Polygon;

// this is algorithm is adapted from pseudo code in wiki

const double epsilon = 1e-6;

double inline sqr(double x) {
    return x*x;
}

bool inline isClose(double a, double b) {
    return abs(a-b) <= epsilon;
}

double inline dist(const Point &u, const Point &v) {
    return sqr(u.real() - v.real()) + sqr(u.imag() - v.imag());
}

double inline cosine(const Point &u, const Point &v) {
    double detX = u.real() - v.real();
    double detY = u.imag() - v.imag();
    return isClose(detX, 0) && isClose(detY, 0) ? 0 : detX / sqrt(sqr(detX) + sqr(detY));
}

struct Compare {
	Point p0; // Leftmost point of the poly
	bool operator ()(const Point &p1, const Point &p2) const {
	    double cos_1 = cosine(p0, p1);
	    double cos_2 = cosine(p0, p2);
        return isClose(cos_1, cos_2) ? dist(p0, p1) < dist(p0, p2) : cos_1 < cos_2;
	}
};

// if 2 vectors are counterclockwise (salient angle)
bool inline salientAngle(Point &a, Point &b, Point &c) {
    return (b.real() - a.real())*(c.imag() - a.imag()) - (b.imag() - a.imag())*(c.real() - a.real()) > 0;
}

////////////////////////////////////////////////////////////////////////////////

Polygon convex_hull(std::vector<Point> &points) {
    Polygon hull;
    if (points.empty()) {
        return hull;
    }

	Compare order;
	// find low-left point as p0 (not original point!)
	auto it = std::min_element(points.begin(), points.end(), [](Point &a, Point &b) {
	   return isClose(a.imag(), b.imag()) ? a.real() < b.real() : a.imag() < b.imag();
	});
	order.p0 = *it;
	std::sort(points.begin(), points.end(), order);

	// use salientAngle(a, b, c) here
	// graham algorithm
	for (auto &p : points) {
	    while (hull.size() > 1 && !salientAngle(hull.rbegin()[1], hull.rbegin()[0], p)) {
            hull.pop_back();
	    }
	    hull.push_back(p);
	}

	return hull;
}

////////////////////////////////////////////////////////////////////////////////

// cuz of NRVO, return value will be fine
std::vector<Point> load_xyz(const std::string &filename) {
	std::vector<Point> points;
	std::ifstream in(filename);
	int n;
	in >> n;
	while (n--) {
	    double x, y, z;
	    in >> x >> y >> z;
	    points.emplace_back(x, y);
	}
	return points;
}

void save_obj(const std::string &filename, Polygon &poly) {
	std::ofstream out(filename);
	if (!out.is_open()) {
		throw std::runtime_error("failed to open file " + filename);
	}
	out << std::fixed;
	for (const auto &v : poly) {
		out << "v " << v.real() << ' ' << v.imag() << " 0\n";
	}
	for (size_t i = 0; i < poly.size(); ++i) {
		out << "l " << i+1 << ' ' << 1+(i+1)%poly.size() << "\n";
	}
	out << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char * argv[]) {
	if (argc <= 2) {
		std::cerr << "Usage: " << argv[0] << " points.xyz output.obj" << std::endl;
	}
	std::vector<Point> points = load_xyz(argv[1]);
	Polygon hull = convex_hull(points);
	save_obj(argv[2], hull);
	return 0;
}
