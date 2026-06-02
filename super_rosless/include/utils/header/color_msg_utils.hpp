#pragma once

class Color {
 public:
  double r{0.0};
  double g{0.0};
  double b{0.0};
  double a{1.0};

  Color() = default;

  explicit Color(int hex_color) {
    const int rr = (hex_color >> 16) & 0xFF;
    const int gg = (hex_color >> 8) & 0xFF;
    const int bb = hex_color & 0xFF;
    r = static_cast<double>(rr) / 255.0;
    g = static_cast<double>(gg) / 255.0;
    b = static_cast<double>(bb) / 255.0;
  }

  Color(const Color& c, double alpha) : r(c.r), g(c.g), b(c.b), a(alpha) {}

  Color(double red, double green, double blue) : Color(red, green, blue, 1.0) {}

  Color(double red, double green, double blue, double alpha) {
    r = red > 1.0 ? red / 255.0 : red;
    g = green > 1.0 ? green / 255.0 : green;
    b = blue > 1.0 ? blue / 255.0 : blue;
    a = alpha;
  }

  static Color White() { return Color(1.0, 1.0, 1.0); }
  static Color Black() { return Color(0.0, 0.0, 0.0); }
  static Color Gray() { return Color(0.5, 0.5, 0.5); }
  static Color Red() { return Color(1.0, 0.0, 0.0); }
  static Color Green() { return Color(0.0, 0.96, 0.0); }
  static Color Blue() { return Color(0.0, 0.0, 1.0); }
  static Color SteelBlue() { return Color(0.4, 0.7, 1.0); }
  static Color Yellow() { return Color(1.0, 1.0, 0.0); }
  static Color Orange() { return Color(1.0, 0.5, 0.0); }
  static Color Purple() { return Color(0.5, 0.0, 1.0); }
  static Color Chartreuse() { return Color(0.5, 1.0, 0.0); }
  static Color Teal() { return Color(0.0, 1.0, 1.0); }
  static Color Pink() { return Color(1.0, 0.0, 0.5); }
};
