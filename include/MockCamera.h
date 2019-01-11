#pragma once
// C++ Libraries
#include <filesystem>
#include <boost/filesystem.hpp>
#include <iostream>

// OpenCV Libraries
#include "Version.h"
#include <opencv2/core.hpp>
#include <librealsense2/rs.hpp>

// OpenARK Libraries
#include "DepthCamera.h"

namespace ark {
	/**
	 * This class defines the behavior of a camera that reads from a data file rather than a live camera
	**/
	class MockCamera : public DepthCamera
	{
	public:
		explicit MockCamera(std::string& file_path);

		int getHeight() const override;

		int getWidth() const override;

		~MockCamera();

	protected:
		/**
		* Gets the new frame from the sensor (implements functionality).
		* Updates xyzMap and ir_map.
		*/
		void update(MultiCameraFrame & frame) override;

	private:
		int height;
		int width;
		std::deque<std::string> file_names;
	};
}
