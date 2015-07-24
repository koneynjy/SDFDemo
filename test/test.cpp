#include <Windows.h>
#include <DirectXMath.h>
#include <iostream>
#include <cmath>
using namespace std;
using namespace DirectX;
#define TEST_COUNT 100000000

inline int myfloorf(){

}

__forceinline float Q_rsqrt(float number)
{
	long i;
	float x2, y;
	const float threehalfs = 1.5F;

	x2 = number * 0.5F;
	y = number;
	i = *(long *)&y;                       // evil floating point bit level hacking（对浮点数的邪恶位级hack）
	i = 0x5f375a86 - (i >> 1);               // what the fuck?（这他妈的是怎么回事？）
	y = *(float *)&i;
	y = y * (threehalfs - (x2 * y * y));   // 1st iteration （第一次牛顿迭代）
	//	y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed（第二次迭代，可以删除）
	return y;
}

__forceinline DirectX::XMVECTOR QuickNormalize(DirectX::XMVECTOR& v){
	return XMVectorScale(v, Q_rsqrt(XMVector3Dot(v, v).m128_f32[0]));
}

struct float2{
	float2(float xx, float yy) :x(xx), y(yy){}
	float2 operator*(float f){
		return float2(x*f, y*f);
	}

	float2 operator/(float f){
		return float2(x/f, y/f);
	}
	float x, y;
};

float2 floor(float2& f2){
	return float2(floorf(f2.x), floorf(f2.y));
}

float2 frac(float2& f2){
	return float2(abs(f2.x - int(f2.x)), abs(f2.y - int(f2.y)));
}

struct float3{
	float3(float xx, float yy, float zz) :x(xx), y(yy), z(zz){}
	float3(float2& f2, float f) :x(f2.x), y(f2.y), z(f){};
	float3 operator*(float f){
		return float3(x*f, y*f, z*f);
	}

	float3 operator/(float f){
		return float3(x / f, y / f, z / f);
	}
	float x, y, z;
};

struct float4{
	float4(float3& f3, float f) :x(f3.x), y(f3.y), z(f3.z), w(f){};
	float x, y, z, w;
};

int main(void)
{
// 	__int64 countsPerSec;
// 	QueryPerformanceFrequency((LARGE_INTEGER*)&countsPerSec);
// 	double mSecondsPerCount = 1.0 / (double)countsPerSec;
// 	__int64 startTime;
// 	XMVECTOR v0 = { 1.5f, 1.5f, 1.5f }, v1, v2, v3;
// 	QueryPerformanceCounter((LARGE_INTEGER*)&startTime);
// 	for (int j = 0; j < TEST_COUNT; j++){
// 		v1 = XMVector3Normalize(v0);
// 	}
// 	__int64 currTime;
// 	cout << v1.m128_f32[0] << endl;
// 	QueryPerformanceCounter((LARGE_INTEGER*)&currTime);
// 
// 	cout << "TIME 01:" << (currTime - startTime) * mSecondsPerCount << endl;
// 
// 	QueryPerformanceCounter((LARGE_INTEGER*)&startTime);
// 	for (int j = 0; j < TEST_COUNT; j++){
// 		v2 = QuickNormalize(v0);;
// 	}
// 	QueryPerformanceCounter((LARGE_INTEGER*)&currTime);
// 	cout << v2.m128_f32[0] << endl;
// 	cout << "TIME 02:" << (currTime - startTime) * mSecondsPerCount << endl;



}

