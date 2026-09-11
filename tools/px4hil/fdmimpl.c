// cgo compiles C only inside the package dir; one shim per translation
// unit (single-TU inclusion would collide the two static namespaces).
#include "fdm.c"
