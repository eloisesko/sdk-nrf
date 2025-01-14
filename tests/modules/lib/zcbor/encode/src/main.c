/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <ztest.h>
#include "test_encode.h"

static void test_encode(void)
{
	uint8_t payload[32];
	uint32_t payload_len;

	uint8_t first_name[] = "Foo";
	uint8_t last_name[] = "Bar";
	uint8_t time[] = {1, 2, 3, 4, 5, 6, 7, 8};
	struct Test test = {
		._Test_name_tstr = {
			{.value = first_name, .len = sizeof(first_name) - 1},
			{.value = last_name, .len = sizeof(last_name) - 1},
		},
		._Test_name_tstr_count = 2,
		._Test_time = {.value = time, .len = sizeof(time)},
		._Test_types_choice = _Test_types_something,
	};

	int res = cbor_encode_Test(payload, sizeof(payload), &test, &payload_len);

	zassert_equal(ZCBOR_SUCCESS, res, "Encoding should have been successful\n");
}

void test_main(void)
{
	ztest_test_suite(lib_zcbor_test,
	     ztest_unit_test(test_encode)
	 );

	ztest_run_test_suite(lib_zcbor_test);
}
