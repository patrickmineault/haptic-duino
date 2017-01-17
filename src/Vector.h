#include <math.h>

// Vector struct.
struct Vector {
  float x;
  float y;
  float z;
};

float dot(Vector v, Vector w) {
  return v.x * w.x + v.y * w.y + v.z * w.z;
}

Vector crossProduct(Vector v, Vector w) {
  Vector u = Vector();
  u.x =  v.y * w.z - v.z * w.y;
  u.y = -v.x * w.z + v.z * w.x;
  u.z =  v.x * w.y - v.y * w.x;
  return u;
}

Vector projectOut(Vector v, Vector w) {
  float proj = dot(v, w);
  v.x -= proj * w.x;
  v.y -= proj * w.y;
  v.z -= proj * w.z;
  return v;
}

float norm(Vector v) {
  return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

Vector normalize(Vector v) {
  float n = norm(v);
  v.x /= n;
  v.y /= n;
  v.z /= n;
  return v;
}
