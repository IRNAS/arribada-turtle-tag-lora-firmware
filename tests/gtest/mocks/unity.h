#ifndef _UNITY_H_
#define _UNITY_H_

#include <assert.h>
#include <string.h>
#include <stdio.h>

#define TEST_LINE_NUM __LINE__
#define UNITY_LINE_TYPE unsigned long

#define UNITY_TEST_ASSERT(condition, line, message)                               if (!(condition)) { printf(message "\n"); assert(condition); }

#define UNITY_TEST_ASSERT_NULL(pointer, line, message)                            if ((NULL != (pointer))) { printf(message "\n"); assert((pointer) == NULL); }

#define UNITY_TEST_ASSERT_NOT_NULL(pointer, line, message)                        if ((NULL == (pointer))) { printf(message "\n"); assert((pointer) != NULL); }

#define UNITY_TEST_ASSERT_EQUAL_INT(expected, actual, line, message)              if ((expected) != (actual)) { printf(message "\n"); assert((expected) == (actual)); }

#define UNITY_TEST_ASSERT_EQUAL_STRING(expected, actual, line, message)           if (strcmp((expected), (actual)) != 0) { printf(message "\n"); assert(strcmp(expected, actual) == 0); }

#define UNITY_TEST_ASSERT_EQUAL_MEMORY(expected, actual, len, line, message)      if(memcmp(expected, actual, len) != 0) { printf(message "\n%d\n", memcmp(expected, actual, len)); assert(memcmp(expected, actual, len) == 0); }

#define UNITY_TEST_ASSERT_EQUAL_STRING_ARRAY(expected, actual, num_elements, line, message)                                                                       for (int _unity_cnt = 0; _unity_cnt < num_elements; _unity_cnt++) { if (!strcmp(expected[_unity_cnt], actual[_unity_cnt])) { printf(message "at element: %d\n", _unity_cnt); assert(strcmp(expected[_unity_cnt], actual[_unity_cnt]) == 0); } }

#define UNITY_TEST_ASSERT_EQUAL_INT_ARRAY(expected, actual, num_elements, line, message)                                                                          for (int _unity_cnt = 0; _unity_cnt < num_elements; _unity_cnt++) { if (expected[_unity_cnt] != actual[_unity_cnt]) { printf(message " at element: %d\n", _unity_cnt); assert(expected[_unity_cnt] == actual[_unity_cnt]); } }

#define UNITY_TEST_ASSERT_EQUAL_HEX32_ARRAY UNITY_TEST_ASSERT_EQUAL_INT_ARRAY
#define UNITY_TEST_ASSERT_EQUAL_HEX16_ARRAY UNITY_TEST_ASSERT_EQUAL_INT_ARRAY
#define UNITY_TEST_ASSERT_EQUAL_HEX8_ARRAY UNITY_TEST_ASSERT_EQUAL_INT_ARRAY
#define UNITY_TEST_ASSERT_EQUAL_HEX32 UNITY_TEST_ASSERT_EQUAL_INT
#define UNITY_TEST_ASSERT_EQUAL_HEX16 UNITY_TEST_ASSERT_EQUAL_INT
#define UNITY_TEST_ASSERT_EQUAL_HEX8 UNITY_TEST_ASSERT_EQUAL_INT
#define UNITY_TEST_ASSERT_EQUAL_PTR UNITY_TEST_ASSERT_EQUAL_INT

#endif
