#pragma once
void unity_output_char(char c);
void unity_output_flush(void);
#define UNITY_OUTPUT_CHAR(a)  unity_output_char(a)
#define UNITY_OUTPUT_FLUSH()  unity_output_flush()
