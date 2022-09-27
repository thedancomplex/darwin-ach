/*
 *   Camera.h
 *
 *   Author: ROBOTIS
 *
 */

#ifndef _CAMERA_H_
#define _CAMERA_H_


namespace Robot
{
	class Camera
	{
	public:
		constexpr static const double VIEW_V_ANGLE = 46.0; //degree
		constexpr static const double VIEW_H_ANGLE = 58.0; //degree

		static int WIDTH;
		static int HEIGHT;
	};

}

#endif
