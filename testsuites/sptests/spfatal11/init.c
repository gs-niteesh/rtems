#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "../spfatal_support/spfatal.h"

/*  Test __assert_func with NULL function name
 *
 *  COPYRIGHT (c) 1989-2009.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#include <assert.h>
#include <string.h>

#define FATAL_ERROR_TEST_NAME            "11"
#define FATAL_ERROR_DESCRIPTION          "asserting with NULL strings..."
#define FATAL_ERROR_EXPECTED_SOURCE      RTEMS_FATAL_SOURCE_ASSERT
#define FATAL_ERROR_EXPECTED_ERROR_CHECK spfatal11_is_expected_error

#define ASSERT_FILE "testcase.h"
#define ASSERT_LINE 38
#define ASSERT_FUNC NULL
#define ASSERT_FEXP "forced"

static inline bool spfatal11_is_expected_error( rtems_fatal_code error )
{
  const rtems_assert_context *assert_context =
    (const rtems_assert_context *) error;

  return strcmp( assert_context->file, ASSERT_FILE ) == 0
    && assert_context->line == ASSERT_LINE
    && assert_context->function == ASSERT_FUNC
    && strcmp( assert_context->failed_expression, ASSERT_FEXP ) == 0;
}

static void force_error(void)
{
  __assert_func( ASSERT_FILE, ASSERT_LINE, ASSERT_FUNC, ASSERT_FEXP );

  /* we will not run this far */
}

#include "../spfatal_support/spfatalimpl.h"
