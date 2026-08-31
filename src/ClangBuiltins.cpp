
//#include <BootLib/Cpu.h>

#include <stdint.h>
#include <limits.h>

namespace Cpu { [[noreturn]] void Panic(char const* fmt, ...); }

#define COMPILER_RT_ABI extern "C" __attribute__((visibility("default")))

#define UNUSED [[maybe_unused]]

// si_int is defined in Linux sysroot's asm-generic/siginfo.h
#ifdef si_int
#undef si_int
#endif
typedef int32_t si_int;
typedef uint32_t su_int;
#if UINT_MAX == 0xFFFFFFFF
#define clzsi __builtin_clz
#define ctzsi __builtin_ctz
#elif ULONG_MAX == 0xFFFFFFFF
#define clzsi __builtin_clzl
#define ctzsi __builtin_ctzl
#else
#error could not determine appropriate clzsi macro for this system
#endif

typedef int64_t di_int;
typedef uint64_t du_int;

typedef union {
  di_int all;
  struct {
#if _YUGA_LITTLE_ENDIAN
    su_int low;
    si_int high;
#else
    si_int high;
    su_int low;
#endif // _YUGA_LITTLE_ENDIAN
  } s;
} dwords;

typedef union {
  du_int all;
  struct {
#if _YUGA_LITTLE_ENDIAN
    su_int low;
    su_int high;
#else
    su_int high;
    su_int low;
#endif // _YUGA_LITTLE_ENDIAN
  } s;
} udwords;

#if defined(__LP64__) || defined(__wasm__) || defined(__mips64) ||             \
    defined(__SIZEOF_INT128__) || defined(_WIN64)
#define CRT_HAS_128BIT
#endif

// MSVC doesn't have a working 128bit integer type. Users should really compile
// compiler-rt with clang, but if they happen to be doing a standalone build for
// asan or something else, disable the 128 bit parts so things sort of work.
#if defined(_MSC_VER) && !defined(__clang__)
#undef CRT_HAS_128BIT
#endif

// The core SPIR-V specification does not support 128-bit integers.
#if defined(__SPIRV__)
#undef CRT_HAS_128BIT
#endif

#ifdef CRT_HAS_128BIT
typedef int ti_int __attribute__((mode(TI)));
typedef unsigned tu_int __attribute__((mode(TI)));

typedef union {
  ti_int all;
  struct {
#if _YUGA_LITTLE_ENDIAN
    du_int low;
    di_int high;
#else
    di_int high;
    du_int low;
#endif // _YUGA_LITTLE_ENDIAN
  } s;
} twords;

typedef union {
  tu_int all;
  struct {
#if _YUGA_LITTLE_ENDIAN
    du_int low;
    du_int high;
#else
    du_int high;
    du_int low;
#endif // _YUGA_LITTLE_ENDIAN
  } s;
} utwords;

static __inline ti_int make_ti(di_int h, di_int l) {
  twords r;
  r.s.high = (du_int)h;
  r.s.low = (du_int)l;
  return r.all;
}

static __inline tu_int make_tu(du_int h, du_int l) {
  utwords r;
  r.s.high = h;
  r.s.low = l;
  return r.all;
}

#endif // CRT_HAS_128BIT

// FreeBSD's boot environment does not support using floating-point and poisons
// the float and double keywords.
#if defined(__FreeBSD__) && defined(_STANDALONE)
#define CRT_HAS_FLOATING_POINT 0
#else
#define CRT_HAS_FLOATING_POINT 1
#endif

#if CRT_HAS_FLOATING_POINT
typedef union {
  su_int u;
  float f;
} float_bits;

typedef union {
  udwords u;
  double f;
} double_bits;

typedef struct {
#if _YUGA_LITTLE_ENDIAN
  udwords low;
  udwords high;
#else
  udwords high;
  udwords low;
#endif // _YUGA_LITTLE_ENDIAN
} uqwords;

// Check if the target supports 80 bit extended precision long doubles.
// Notably, on x86 Windows, MSVC only provides a 64-bit long double, but GCC
// still makes it 80 bits. Clang will match whatever compiler it is trying to
// be compatible with. On 32-bit x86 Android, long double is 64 bits, while on
// x86_64 Android, long double is 128 bits.
#if (defined(__i386__) || defined(__x86_64__)) &&                              \
    !(defined(_MSC_VER) || defined(__ANDROID__))
#define HAS_80_BIT_LONG_DOUBLE 1
#elif defined(__m68k__) || defined(__ia64__)
#define HAS_80_BIT_LONG_DOUBLE 1
#else
#define HAS_80_BIT_LONG_DOUBLE 0
#endif

#if HAS_80_BIT_LONG_DOUBLE
typedef long double xf_float;
typedef union {
  uqwords u;
  xf_float f;
} xf_bits;
#endif

#ifdef __powerpc64__
// From https://gcc.gnu.org/wiki/Ieee128PowerPC:
// PowerPC64 uses the following suffixes:
// IFmode: IBM extended double
// KFmode: IEEE 128-bit floating point
// TFmode: Matches the default for long double. With -mabi=ieeelongdouble,
//         it is IEEE 128-bit, with -mabi=ibmlongdouble IBM extended double
// Since compiler-rt only implements the tf set of libcalls, we use long double
// for the tf_float typedef.
typedef long double tf_float;
#define CRT_LDBL_128BIT
#define CRT_HAS_F128
#if __LDBL_MANT_DIG__ == 113 && !defined(__LONG_DOUBLE_IBM128__)
#define CRT_HAS_IEEE_TF
#define CRT_LDBL_IEEE_F128
#endif
#define TF_C(x) x##L
#elif __LDBL_MANT_DIG__ == 113 ||                                              \
    (__FLT_RADIX__ == 16 && __LDBL_MANT_DIG__ == 28)
// Use long double instead of __float128 if it matches the IEEE 128-bit format
// or the IBM hexadecimal format.
#define CRT_LDBL_128BIT
#define CRT_HAS_F128
#if __LDBL_MANT_DIG__ == 113
#define CRT_HAS_IEEE_TF
#define CRT_LDBL_IEEE_F128
#endif
typedef long double tf_float;
#define TF_C(x) x##L
#elif defined(__FLOAT128__) || defined(__SIZEOF_FLOAT128__)
#define CRT_HAS___FLOAT128_KEYWORD
#define CRT_HAS_F128
// NB: we assume the __float128 type uses IEEE representation.
#define CRT_HAS_IEEE_TF
typedef __float128 tf_float;
#define TF_C(x) x##Q
#endif

#ifdef CRT_HAS_F128
typedef union {
  uqwords u;
  tf_float f;
} tf_bits;
#endif

// __(u)int128_t is currently needed to compile the *tf builtins as we would
// otherwise need to manually expand the bit manipulation on two 64-bit value.
#if defined(CRT_HAS_128BIT) && defined(CRT_HAS_F128)
#define CRT_HAS_TF_MODE
#endif

#if __STDC_VERSION__ >= 199901L && !defined(_MSC_VER)
typedef float _Complex Fcomplex;
typedef double _Complex Dcomplex;
typedef long double _Complex Lcomplex;
#if defined(CRT_LDBL_128BIT)
typedef Lcomplex Qcomplex;
#define CRT_HAS_NATIVE_COMPLEX_F128
#elif defined(CRT_HAS___FLOAT128_KEYWORD)
#if defined(__clang_major__) && __clang_major__ > 10
// Clang prior to 11 did not support __float128 _Complex.
typedef __float128 _Complex Qcomplex;
#define CRT_HAS_NATIVE_COMPLEX_F128
#elif defined(__GNUC__) && __GNUC__ >= 7
// GCC does not allow __float128 _Complex, but accepts _Float128 _Complex.
typedef _Float128 _Complex Qcomplex;
#define CRT_HAS_NATIVE_COMPLEX_F128
#endif
#endif

#define COMPLEX_REAL(x) __real__(x)
#define COMPLEX_IMAGINARY(x) __imag__(x)
#else
typedef struct {
  float real, imaginary;
} Fcomplex;

typedef struct {
  double real, imaginary;
} Dcomplex;

typedef struct {
  long double real, imaginary;
} Lcomplex;

#define COMPLEX_REAL(x) (x).real
#define COMPLEX_IMAGINARY(x) (x).imaginary
#endif

#ifdef CRT_HAS_NATIVE_COMPLEX_F128
#define COMPLEXTF_REAL(x) __real__(x)
#define COMPLEXTF_IMAGINARY(x) __imag__(x)
#elif defined(CRT_HAS_F128)
typedef struct {
  tf_float real, imaginary;
} Qcomplex;
#define COMPLEXTF_REAL(x) (x).real
#define COMPLEXTF_IMAGINARY(x) (x).imaginary
#endif

#endif // CRT_HAS_FLOATING_POINT

// Returns the 128 bit division result by 64 bit. Result must fit in 64 bits.
// Remainder stored in r.
// Taken and adjusted from libdivide libdivide_128_div_64_to_64 division
// fallback. For a correctness proof see the reference for this algorithm
// in Knuth, Volume 2, section 4.3.1, Algorithm D.
UNUSED
static inline du_int udiv128by64to64default(du_int u1, du_int u0, du_int v,
                                            du_int *r) {
  const unsigned n_udword_bits = sizeof(du_int) * CHAR_BIT;
  const du_int b = (1ULL << (n_udword_bits / 2)); // Number base (32 bits)
  du_int un1, un0;                                // Norm. dividend LSD's
  du_int vn1, vn0;                                // Norm. divisor digits
  du_int q1, q0;                                  // Quotient digits
  du_int un64, un21, un10;                        // Dividend digit pairs
  du_int rhat;                                    // A remainder
  si_int s;                                       // Shift amount for normalization

  s = __builtin_clzll(v);
  if (s > 0) {
    // Normalize the divisor.
    v = v << s;
    un64 = (u1 << s) | (u0 >> (n_udword_bits - s));
    un10 = u0 << s; // Shift dividend left
  } else {
    // Avoid undefined behavior of (u0 >> 64).
    un64 = u1;
    un10 = u0;
  }

  // Break divisor up into two 32-bit digits.
  vn1 = v >> (n_udword_bits / 2);
  vn0 = v & 0xFFFFFFFF;

  // Break right half of dividend into two digits.
  un1 = un10 >> (n_udword_bits / 2);
  un0 = un10 & 0xFFFFFFFF;

  // Compute the first quotient digit, q1.
  q1 = un64 / vn1;
  rhat = un64 - q1 * vn1;

  // q1 has at most error 2. No more than 2 iterations.
  while (q1 >= b || q1 * vn0 > b * rhat + un1) {
    q1 = q1 - 1;
    rhat = rhat + vn1;
    if (rhat >= b)
      break;
  }

  un21 = un64 * b + un1 - q1 * v;

  // Compute the second quotient digit.
  q0 = un21 / vn1;
  rhat = un21 - q0 * vn1;

  // q0 has at most error 2. No more than 2 iterations.
  while (q0 >= b || q0 * vn0 > b * rhat + un0) {
    q0 = q0 - 1;
    rhat = rhat + vn1;
    if (rhat >= b)
      break;
  }

  *r = (un21 * b + un0 - q0 * v) >> s;
  return q1 * b + q0;
}

static inline du_int udiv128by64to64(du_int u1, du_int u0, du_int v,
                                     du_int *r) {
#if defined(__x86_64__) && !defined(__arm64ec__)
  du_int result;
  __asm__("divq %[v]"
          : "=a"(result), "=d"(*r)
          : [ v ] "r"(v), "a"(u0), "d"(u1));
  return result;
#else
  return udiv128by64to64default(u1, u0, v, r);
#endif
}

// Effects: if rem != 0, *rem = a % b
// Returns: a / b

COMPILER_RT_ABI tu_int __udivmodti4(tu_int a, tu_int b, tu_int *rem) {
  const unsigned n_utword_bits = sizeof(tu_int) * CHAR_BIT;
  utwords dividend;
  dividend.all = a;
  utwords divisor;
  divisor.all = b;
  utwords quotient;
  utwords remainder;
  if (divisor.all > dividend.all) {
    if (rem)
      *rem = dividend.all;
    return 0;
  }
  // When the divisor fits in 64 bits, we can use an optimized path.
  if (divisor.s.high == 0) {
    remainder.s.high = 0;
    if (dividend.s.high < divisor.s.low) {
      // The result fits in 64 bits.
      quotient.s.low = udiv128by64to64(dividend.s.high, dividend.s.low,
                                       divisor.s.low, &remainder.s.low);
      quotient.s.high = 0;
    } else {
      // First, divide with the high part to get the remainder in dividend.s.high.
      // After that dividend.s.high < divisor.s.low.
      quotient.s.high = dividend.s.high / divisor.s.low;
      dividend.s.high = dividend.s.high % divisor.s.low;
      quotient.s.low = udiv128by64to64(dividend.s.high, dividend.s.low,
                                       divisor.s.low, &remainder.s.low);
    }
    if (rem)
      *rem = remainder.all;
    return quotient.all;
  }
  // 0 <= shift <= 63.
  si_int shift =
      __builtin_clzll(divisor.s.high) - __builtin_clzll(dividend.s.high);
  divisor.all <<= shift;
  quotient.s.high = 0;
  quotient.s.low = 0;
  for (; shift >= 0; --shift) {
    quotient.s.low <<= 1;
    // Branch free version of.
    // if (dividend.all >= divisor.all)
    // {
    //    dividend.all -= divisor.all;
    //    carry = 1;
    // }
    const ti_int s =
        (ti_int)(divisor.all - dividend.all - 1) >> (n_utword_bits - 1);
    quotient.s.low |= s & 1;
    dividend.all -= divisor.all & s;
    divisor.all >>= 1;
  }
  if (rem)
    *rem = dividend.all;
  return quotient.all;
}

COMPILER_RT_ABI tu_int __udivti3(tu_int a, tu_int b) {
  return __udivmodti4(a, b, 0);
}

COMPILER_RT_ABI tu_int __umodti3(tu_int a, tu_int b) {
  tu_int r;
  __udivmodti4(a, b, &r);
  return r;
}

using fp_t = long double;
using CMP_RESULT = int;

COMPILER_RT_ABI CMP_RESULT __unordtf2(fp_t a, fp_t b) {
    Cpu::Panic("unordtf2 not implemented");
//  if (a == b) return 0;
//  return (a < b) ? -1 : 1;
}
