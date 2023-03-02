#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <opencv2/core/core.hpp>

namespace py = pybind11;

namespace pybind11 { namespace detail{
template<>
struct type_caster<cv::Rect>{
    PYBIND11_TYPE_CASTER(cv::Rect, _("tuple_xywh"));

    bool load(handle obj, bool){
        if(!py::isinstance<py::tuple>(obj)){
            std::logic_error("Rect should be a tuple!");
            return false;
        }
        py::tuple rect = reinterpret_borrow<py::tuple>(obj);
        if(rect.size()!=4){
            std::logic_error("Rect (x,y,w,h) tuple should be size of 4");
            return false;
        }

        value = cv::Rect(rect[0].cast<int>(), rect[1].cast<int>(), rect[2].cast<int>(), rect[3].cast<int>());
        return true;
    }

    static handle cast(const cv::Rect& rect, return_value_policy, handle){
        return py::make_tuple(rect.x, rect.y, rect.width, rect.height).release();
    }
};

}} //!  end namespace pybind11::detail
