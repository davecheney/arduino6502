#include "Arduino.h"
#include "cpu.hpp"
#include "rom.h"


namespace CPU {


/* CPU state */
u8 ram[0x1000];
u8 A, X, Y, S;
u16 PC;
Flags P;
bool nmi, irq;

/* Cycle emulation */
#define T   tick()

byte v = HIGH;
inline void tick() {
  // PORTB |= (1 << PORTB7);
  // PORTB &= ~(1 << PORTB7);
}

/* Flags updating */
inline void upd_cv(u8 x, u8 y, s16 r) {
  P[C] = (r > 0xFF);
  P[V] = ~(x ^ y) & (x ^ r) & 0x80;
}
inline void upd_nz(u8 x)              {
  P[N] = x & 0x80;
  P[Z] = (x == 0);
}
// Does adding I to A cross a page?
inline bool cross(u16 a, u8 i) {
  return ((a + i) & 0xFF00) != ((a & 0xFF00));
}

/* Memory access */
template<bool wr> inline u8 access(u16 addr, u8 v = 0)
{
  u8* r;
  if (wr) {
    PORTB &= ~(1 << PORTB6);
  } else {
    PORTB |= (1 << PORTB6);
  }
  switch (addr) {
    case 0x0000 ... 0x0FFF:  r = &ram[addr]; if (wr) *r = v; return *r;  // RAM.
    case 0xe000 ... 0xefff:  return pgm_read_byte(erom + (addr - 0xe000));
    case 0xf000 ... 0xffff:  return pgm_read_byte(from + (addr - 0xf000));
    case 0xd010:
      return 0x80 | UDR0;
    case 0xd011:
      return  UCSR0A & _BV(RXC0);
    case 0xd012:
      if (wr) {
        UDR0 = v & 0x7f;
        if ((v & 0x7f) == '\r') {
          loop_until_bit_is_set(UCSR0A, UDRE0);
          UDR0 = '\n';
        }
      } else {
        return UCSR0A & _BV(UDRE0) ? 0 : 0x80;
      }
  }
  return 0;
}
inline u8  wr(u16 a, u8 v)      {
  T;
  return access<1>(a, v);
}
inline u8  rd(u16 a)            {
  T;
  return access<0>(a);
}
inline u16 rd16_d(u16 a, u16 b) {
  return rd(a) | (rd(b) << 8);  // Read from A and B and merge.
}
inline u16 rd16(u16 a)          {
  return rd16_d(a, a + 1);
}
inline u8  push(u8 v)           {
  return wr(0x100 + (S--), v);
}
inline u8  pop()                {
  return rd(0x100 + (++S));
}

/* Addressing modes */
inline u16 imm()   {
  return PC++;
}
inline u16 imm16() {
  PC += 2;
  return PC - 2;
}
inline u16 _abs()   {
  return rd16(imm16());
}
inline u16 _abx()  {
  T;  // Exception.
  return _abs() + X;
}
inline u16 abx()   {
  u16 a = _abs();
  if (cross(a, X)) T;
  return a + X;
}
inline u16 aby()   {
  u16 a = _abs();
  if (cross(a, Y)) T;
  return a + Y;
}
inline u16 zp()    {
  return rd(imm());
}
inline u16 zpx()   {
  T;
  return (zp() + X) % 0x100;
}
inline u16 zpy()   {
  T;
  return (zp() + Y) % 0x100;
}
inline u16 izx()   {
  u8 i = zpx();
  return rd16_d(i, (i + 1) % 0x100);
}
inline u16 _izy()  {
  u8 i = zp();   // Exception.
  return rd16_d(i, (i + 1) % 0x100) + Y;
}
inline u16 izy()   {
  u16 a = _izy();
  if (cross(a - Y, Y)) T;
  return a;
}

/* STx */
template<u8& r, Mode m> void st()        {
  wr(   m()    , r);
}
template<>              void st<A, izy>() {
  T;  // Exceptions.
  wr(_izy()    , A);
}
template<>              void st<A, abx>() {
  T;  // ...
  wr( _abs() + X, A);
}
template<>              void st<A, aby>() {
  T;  // ...
  wr( _abs() + Y, A);
}

#define G  u16 a = m(); u8 p = rd(a)  /* Fetch parameter */
template<u8& r, Mode m> void ld()  {
  G;  // LDx
  upd_nz(r = p);
}
template<u8& r, Mode m> void cmp() {
  G;  // CMP, CPx
  upd_nz(r - p);
  P[C] = (r >= p);
}
/* Arithmetic and bitwise */
template<Mode m> void _ADC() {
  G       ;
  s16 r = A + p + P[C];
  upd_cv(A, p, r);
  upd_nz(A = r);
}
template<Mode m> void SBC() {
  G ^ 0xFF;
  s16 r = A + p + P[C];
  upd_cv(A, p, r);
  upd_nz(A = r);
}
template<Mode m> void BIT() {
  G;
  P[Z] = !(A & p);
  P[N] = p & 0x80;
  P[V] = p & 0x40;
}
template<Mode m> void AND() {
  G;
  upd_nz(A &= p);
}
template<Mode m> void EOR() {
  G;
  upd_nz(A ^= p);
}
template<Mode m> void ORA() {
  G;
  upd_nz(A |= p);
}
/* Read-Modify-Write */
template<Mode m> void ASL() {
  G;
  P[C] = p & 0x80;
  T;
  upd_nz(wr(a, p << 1));
}
template<Mode m> void LSR() {
  G;
  P[C] = p & 0x01;
  T;
  upd_nz(wr(a, p >> 1));
}
template<Mode m> void ROL() {
  G;
  u8 c = P[C]     ;
  P[C] = p & 0x80;
  T;
  upd_nz(wr(a, (p << 1) | c) );
}
template<Mode m> void ROR() {
  G;
  u8 c = P[C] << 7;
  P[C] = p & 0x01;
  T;
  upd_nz(wr(a, c | (p >> 1)) );
}
template<Mode m> void _DEC() {
  G;
  T;
  upd_nz(wr(a, --p));
}
template<Mode m> void INC() {
  G;
  T;
  upd_nz(wr(a, ++p));
}
#undef G

/* DEx, INx */
template<u8& r> void dec() {
  upd_nz(--r);
  T;
}
template<u8& r> void inc() {
  upd_nz(++r);
  T;
}
/* Bit shifting on the accumulator */
void ASL_A() {
  P[C] = A & 0x80;
  upd_nz(A <<= 1);
  T;
}
void LSR_A() {
  P[C] = A & 0x01;
  upd_nz(A >>= 1);
  T;
}
void ROL_A() {
  u8 c = P[C]     ;
  P[C] = A & 0x80;
  upd_nz(A = ((A << 1) | c) );
  T;
}
void ROR_A() {
  u8 c = P[C] << 7;
  P[C] = A & 0x01;
  upd_nz(A = (c | (A >> 1)) );
  T;
}

/* Txx (move values between registers) */
template<u8& s, u8& d> void tr()      {
  upd_nz(d = s);
  T;
}
template<>             void tr<X, S>() {
  S = X;          // TSX, exception.
  T;
}

/* Stack operations */
void PLP() {
  T;
  T;
  P.set(pop());
}
void PHP() {
  T;  // B flag set.
  push(P.get() | (1 << 4));
}
void PLA() {
  T;
  T;
  A = pop();
  upd_nz(A);
}
void PHA() {
  T;
  push(A);
}

/* Flow control (branches, jumps) */
template<Flag f, bool v> void br() {
  s8 j = rd(imm());
  if (P[f] == v) {
    T;
    PC += j;
  }
}
void JMP_IND() {
  u16 i = rd16(imm16());
  PC = rd16_d(i, (i & 0xFF00) | ((i + 1) % 0x100));
}
void JMP()     {
  PC = rd16(imm16());
}
void JSR()     {
  u16 t = PC + 1;
  T;
  push(t >> 8);
  push(t);
  PC = rd16(imm16());
}

/* Return instructions */
void RTS() {
  T;
  T;
  PC = (pop() | (pop() << 8)) + 1;
  T;
}
void RTI() {
  PLP();
  PC =  pop() | (pop() << 8);
}

template<Flag f, bool v> void flag() {
  P[f] = v;  // Clear and set flags.
  T;
}
template<IntType t> void INT()
{
  T; if (t != BRK) T;  // BRK already performed the fetch.
  if (t != RESET)  // Writes on stack are inhibited on RESET.
  {
    push(PC >> 8); push(PC & 0xFF);
    push(P.get() | ((t == BRK) << 4));  // Set B if BRK.
  }
  else {
    S -= 3;
    T;
    T;
    T;
  }
  P[I] = true;
  /*   NMI    Reset    IRQ     BRK  */
  constexpr u16 vect[] = { 0xFFFA, 0xFFFC, 0xFFFE, 0xFFFE };
  PC = rd16(vect[t]);
  if (t == NMI) nmi = false;
}
void NOP() {
  T;
}

/* Execute a CPU instruction */
void exec()
{
  switch (rd(PC++))  // Fetch the opcode.
  {
    // Select the right function to emulate the instruction:
    case 0x00: return INT<BRK>()  ;  case 0x01: return ORA<izx>()  ;
    case 0x05: return ORA<zp>()   ;  case 0x06: return ASL<zp>()   ;
    case 0x08: return PHP()       ;  case 0x09: return ORA<imm>()  ;
    case 0x0A: return ASL_A()     ;  case 0x0D: return ORA<_abs>()  ;
    case 0x0E: return ASL<_abs>()  ;  case 0x10: return br<N, 0>()   ;
    case 0x11: return ORA<izy>()  ;  case 0x15: return ORA<zpx>()  ;
    case 0x16: return ASL<zpx>()  ;  case 0x18: return flag<C, 0>() ;
    case 0x19: return ORA<aby>()  ;  case 0x1D: return ORA<abx>()  ;
    case 0x1E: return ASL<_abx>() ;  case 0x20: return JSR()       ;
    case 0x21: return AND<izx>()  ;  case 0x24: return BIT<zp>()   ;
    case 0x25: return AND<zp>()   ;  case 0x26: return ROL<zp>()   ;
    case 0x28: return PLP()       ;  case 0x29: return AND<imm>()  ;
    case 0x2A: return ROL_A()     ;  case 0x2C: return BIT<_abs>()  ;
    case 0x2D: return AND<_abs>()  ;  case 0x2E: return ROL<_abs>()  ;
    case 0x30: return br<N, 1>()   ;  case 0x31: return AND<izy>()  ;
    case 0x35: return AND<zpx>()  ;  case 0x36: return ROL<zpx>()  ;
    case 0x38: return flag<C, 1>() ;  case 0x39: return AND<aby>()  ;
    case 0x3D: return AND<abx>()  ;  case 0x3E: return ROL<_abx>() ;
    case 0x40: return RTI()       ;  case 0x41: return EOR<izx>()  ;
    case 0x45: return EOR<zp>()   ;  case 0x46: return LSR<zp>()   ;
    case 0x48: return PHA()       ;  case 0x49: return EOR<imm>()  ;
    case 0x4A: return LSR_A()     ;  case 0x4C: return JMP()       ;
    case 0x4D: return EOR<_abs>()  ;  case 0x4E: return LSR<_abs>()  ;
    case 0x50: return br<V, 0>()   ;  case 0x51: return EOR<izy>()  ;
    case 0x55: return EOR<zpx>()  ;  case 0x56: return LSR<zpx>()  ;
    case 0x58: return flag<I, 0>() ;  case 0x59: return EOR<aby>()  ;
    case 0x5D: return EOR<abx>()  ;  case 0x5E: return LSR<_abx>() ;
    case 0x60: return RTS()       ;  case 0x61: return _ADC<izx>()  ;
    case 0x65: return _ADC<zp>()   ;  case 0x66: return ROR<zp>()   ;
    case 0x68: return PLA()       ;  case 0x69: return _ADC<imm>()  ;
    case 0x6A: return ROR_A()     ;  case 0x6C: return JMP_IND()   ;
    case 0x6D: return _ADC<_abs>()  ;  case 0x6E: return ROR<_abs>()  ;
    case 0x70: return br<V, 1>()   ;  case 0x71: return _ADC<izy>()  ;
    case 0x75: return _ADC<zpx>()  ;  case 0x76: return ROR<zpx>()  ;
    case 0x78: return flag<I, 1>() ;  case 0x79: return _ADC<aby>()  ;
    case 0x7D: return _ADC<abx>()  ;  case 0x7E: return ROR<_abx>() ;
    case 0x81: return st<A, izx>() ;  case 0x84: return st<Y, zp>()  ;
    case 0x85: return st<A, zp>()  ;  case 0x86: return st<X, zp>()  ;
    case 0x88: return dec<Y>()    ;  case 0x8A: return tr<X, A>()   ;
    case 0x8C: return st<Y, _abs>() ;  case 0x8D: return st<A, _abs>() ;
    case 0x8E: return st<X, _abs>() ;  case 0x90: return br<C, 0>()   ;
    case 0x91: return st<A, izy>() ;  case 0x94: return st<Y, zpx>() ;
    case 0x95: return st<A, zpx>() ;  case 0x96: return st<X, zpy>() ;
    case 0x98: return tr<Y, A>()   ;  case 0x99: return st<A, aby>() ;
    case 0x9A: return tr<X, S>()   ;  case 0x9D: return st<A, abx>() ;
    case 0xA0: return ld<Y, imm>() ;  case 0xA1: return ld<A, izx>() ;
    case 0xA2: return ld<X, imm>() ;  case 0xA4: return ld<Y, zp>()  ;
    case 0xA5: return ld<A, zp>()  ;  case 0xA6: return ld<X, zp>()  ;
    case 0xA8: return tr<A, Y>()   ;  case 0xA9: return ld<A, imm>() ;
    case 0xAA: return tr<A, X>()   ;  case 0xAC: return ld<Y, _abs>() ;
    case 0xAD: return ld<A, _abs>() ;  case 0xAE: return ld<X, _abs>() ;
    case 0xB0: return br<C, 1>()   ;  case 0xB1: return ld<A, izy>() ;
    case 0xB4: return ld<Y, zpx>() ;  case 0xB5: return ld<A, zpx>() ;
    case 0xB6: return ld<X, zpy>() ;  case 0xB8: return flag<V, 0>() ;
    case 0xB9: return ld<A, aby>() ;  case 0xBA: return tr<S, X>()   ;
    case 0xBC: return ld<Y, abx>() ;  case 0xBD: return ld<A, abx>() ;
    case 0xBE: return ld<X, aby>() ;  case 0xC0: return cmp<Y, imm>();
    case 0xC1: return cmp<A, izx>();  case 0xC4: return cmp<Y, zp>() ;
    case 0xC5: return cmp<A, zp>() ;  case 0xC6: return _DEC<zp>()   ;
    case 0xC8: return inc<Y>()    ;  case 0xC9: return cmp<A, imm>();
    case 0xCA: return dec<X>()    ;  case 0xCC: return cmp<Y, _abs>();
    case 0xCD: return cmp<A, _abs>();  case 0xCE: return _DEC<_abs>()  ;
    case 0xD0: return br<Z, 0>()   ;  case 0xD1: return cmp<A, izy>();
    case 0xD5: return cmp<A, zpx>();  case 0xD6: return _DEC<zpx>()  ;
    case 0xD8: return flag<D, 0>() ;  case 0xD9: return cmp<A, aby>();
    case 0xDD: return cmp<A, abx>();  case 0xDE: return _DEC<_abx>() ;
    case 0xE0: return cmp<X, imm>();  case 0xE1: return SBC<izx>()  ;
    case 0xE4: return cmp<X, zp>() ;  case 0xE5: return SBC<zp>()   ;
    case 0xE6: return INC<zp>()   ;  case 0xE8: return inc<X>()    ;
    case 0xE9: return SBC<imm>()  ;  case 0xEA: return NOP()       ;
    case 0xEC: return cmp<X, _abs>();  case 0xED: return SBC<_abs>()  ;
    case 0xEE: return INC<_abs>()  ;  case 0xF0: return br<Z, 1>()   ;
    case 0xF1: return SBC<izy>()  ;  case 0xF5: return SBC<zpx>()  ;
    case 0xF6: return INC<zpx>()  ;  case 0xF8: return flag<D, 1>() ;
    case 0xF9: return SBC<aby>()  ;  case 0xFD: return SBC<abx>()  ;
  case 0xFE: return INC<_abx>() ;  default:   return exit(1)     ;
  }
}

void set_nmi(bool v) {
  nmi = v;
}
void set_irq(bool v) {
  irq = v;
}

/* Turn on the CPU */
void power()
{
  P.set(0x04);
  A = X = Y = S = 0x00;
  memset(ram, 0xFF, sizeof(ram));

  nmi = irq = false;
  INT<RESET>();
}

void run() {
  for (;;) {
    if (nmi) INT<NMI>();
    else if (irq and !P[I]) INT<IRQ>();
    exec();
  }
}

}
