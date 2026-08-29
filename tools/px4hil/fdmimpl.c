// cgo compiles C only inside the package dir; one shim per translation
// unit (single-TU inclusion collides the two cordic_port.h variants).
#include "fdm.c"
