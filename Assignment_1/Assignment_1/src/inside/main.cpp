////////////////////////////////////////////////////////////////////////////////
#include <algorithm>
#include <complex>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>
////////////////////////////////////////////////////////////////////////////////

/**
 * https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
 */
const double epsilon = 1e-6;

typedef std::complex<double> Point;
typedef std::vector<Point> Polygon;

bool inline isClose(double a, double b) {
    return abs(a-b) <= epsilon;
}

double inline determinant(const Point &u, const Point &v) {
    return (u.real()*v.imag()) - (u.imag()*v.real());
}

// Return true iff [a,b] intersects [c,d], and store the intersection in ans
bool intersect_segment(const Point &a, const Point &b, const Point &c, const Point &d, Point &ans) {
    Point A = a-c;
    Point B = c-d;
    Point C = a-b;
	double detT = determinant(A, B);
    double detU = -determinant(C, A);
    double det = determinant(C, B);
    if (isClose(det, 0)) {
        return false;
    } else {
        // we skip calculation of intersection, only return if they intersect
        double t = detT/det;
        double u = detU/det;
        bool validT = 0 <= t && t <= 1;
        bool validU = 0 <= u && u <= 1;
        return validT && validU;
    }
}

////////////////////////////////////////////////////////////////////////////////

bool is_inside(const Polygon &poly, const Point &query, const Point &outside) {
	// 2. Cast a ray from the query point to the 'outside' point, count number of intersections
	unsigned int count = 0;
	for (auto i = 0; i < poly.size(); i++) {
	    auto j = (i+1) % poly.size();
	    // i and j define an edge
	    Point interPt;
	    if (intersect_segment(poly[i], poly[j], outside, query, interPt)) {
	        count++;
	    }
	}
	return count%2 == 1;
}

Point calc_outside(const Polygon &poly) {
    // 1. Compute bounding box and set coordinate of a point outside the polygon
    double minX = poly[0].real(), minY = poly[0].imag();
    for (auto &p: poly) {
        if (minX > p.real()) {
            minX = p.real();
        }
        if (minY > p.imag()) {
            minY = p.imag();
        }
    }
//	auto minX = std::min_element(poly.begin(), poly.end(), [](Point &x, Point &y){
//	    return x.real() < y.real();
//	});
//    auto minY = std::min_element(poly.begin(), poly.end(), [](Point &a, Point &b){
//        return a.imag() < b.imag();
//    });
    Point outside(minX - 1000, minY - 1000);
    return outside;
}

////////////////////////////////////////////////////////////////////////////////

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

Polygon load_obj(const std::string &filename) {
	std::ifstream in(filename);
	Polygon poly;
	std::vector<Point> vertices;
    while (!in.eof()) {
        char op = 0;
        double x, y, z;
        in >> op;
        if (op == 'v') {
            in >> x >> y >> z;
            vertices.emplace_back(x, y);
        } else if (op == 'f') {
            std::string the_string;
            std::getline(in, the_string);
            std::istringstream iss(the_string);
            std::vector<int> order;
            for (int n; iss >> n;) {
                order.push_back(n);
            }
            for (auto i : order) {
                poly.push_back(vertices[i-1]);
            }
        } else {
            std::string str;
            std::getline(in, str);
        }
    }
	return poly;
}

void save_xyz(const std::string &filename, const std::vector<Point> &points) {
    std::ofstream out(filename);
    if (!out.is_open()) {
        throw std::runtime_error("failed to open file " + filename);
    }
    out << points.size() << '\n';
    out << std::fixed;
    for (const auto &p : points) {
        out << p.real() << " " << p.imag() << " 0\n";
    }
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char * argv[]) {
	if (argc <= 3) {
		std::cerr << "Usage: " << argv[0] << " points.xyz poly.obj result.xyz" << std::endl;
	}
	std::vector<Point> points = load_xyz(argv[1]);
	Polygon poly = load_obj(argv[2]);
	std::vector<Point> result;
	Point outside = calc_outside(poly);
	for (auto & point : points) {
		if (is_inside(poly, point, outside)) {
			result.push_back(point);
		}
	}
	save_xyz(argv[3], result);
	return 0;
//    Point x;
//    std::cout << intersect_segment(Point(0,0), Point(0,10), Point(3, 4), Point(6, 7), x);
}
