//
// Created by czh on 2021/5/5.
//

#ifndef TCR_WINDMILL_WRAPPER_HEAD_H
#define TCR_WINDMILL_WRAPPER_HEAD_H

#include <opencv2/core/core.hpp>

class WrapperHead {
public:
    virtual ~WrapperHead() = default;;
    virtual bool init() = 0;
    virtual bool read(cv::Mat &src) = 0;
};

#endif //TCR_WINDMILL_WRAPPER_HEAD_H
