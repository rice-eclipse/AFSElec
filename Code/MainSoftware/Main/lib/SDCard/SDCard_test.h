/**
 * SDCard_test.h - Bench test for the SDCard library.
 *
 * Non-destructive: everything happens inside a "_sdtest" folder, which is deleted at the end.
 * The card is never formatted.
 *
 * verbose = false: prints only FAIL lines, diagnostics and the one-line result.
 * verbose = true:  also prints every PASS line, card info, the directory listing and the speed figures.
 *
 * Returns true if every check passed. Leaves the card initialized and mounted.
 */
#pragma once

#include "SDCard.h"

/**
 * @param speedKB size of the write/read speed test in kB (0 skips it)
 */
bool SDCard_test(SDCard& sd, bool verbose = false, uint32_t speedKB = 512);
