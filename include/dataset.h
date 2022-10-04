
#pragma once

//STL
#include <list>
#include <string>

//OpenCV
#include <opencv2/core.hpp>

#include "pinholeStereoCamera.h"

namespace StVO {

class Dataset {
public:

    // Constructor
    Dataset(const std::string &dataset_path, const PinholeStereoCamera &cam, int offset = 0, int nmax = 0, int step = 1);

    // Destrcutor
    virtual ~Dataset();

    // Reads next frame in the dataset sequence, returning true if image was successfully loaded
    bool nextFrame(cv::Mat &img_l, cv::Mat &img_r);

    // Returns if there are images still available in the sequence
    inline bool hasNext();

private:

    std::list<std::string> images_l, images_r;
    const PinholeStereoCamera &cam;
};

} // namespace StVO