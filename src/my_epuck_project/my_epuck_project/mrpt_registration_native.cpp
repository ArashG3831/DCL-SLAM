#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFSOG.h>
#include <mrpt/slam/CGridMapAligner.h>
#include <mrpt/system/COutputLogger.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct MapSpec {
  int width{}, height{};
  double resolution{}, origin_x{}, origin_y{}, origin_yaw{};
  const int16_t* values{};
};

template <typename T>
T integer_argument(PyObject* args, Py_ssize_t index, const char* name)
{
  PyObject* object = PyTuple_GetItem(args, index);
  if (!object) throw std::runtime_error(std::string("missing ") + name);
  const long value = PyLong_AsLong(object);
  if (PyErr_Occurred()) throw std::runtime_error(std::string("invalid ") + name);
  return static_cast<T>(value);
}

double floating_argument(PyObject* args, Py_ssize_t index, const char* name)
{
  PyObject* object = PyTuple_GetItem(args, index);
  if (!object) throw std::runtime_error(std::string("missing ") + name);
  const double value = PyFloat_AsDouble(object);
  if (PyErr_Occurred()) throw std::runtime_error(std::string("invalid ") + name);
  return value;
}

double wrap_pi(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

mrpt::poses::CPose2D inverse_pose(const mrpt::poses::CPose2D& pose)
{
  const double c = std::cos(pose.phi());
  const double s = std::sin(pose.phi());
  return mrpt::poses::CPose2D(
      -c * pose.x() - s * pose.y(),
      s * pose.x() - c * pose.y(),
      wrap_pi(-pose.phi()));
}

mrpt::maps::COccupancyGridMap2D::Ptr make_map(const MapSpec& input)
{
  const float x1 = static_cast<float>(input.origin_x + input.width * input.resolution);
  const float y1 = static_cast<float>(input.origin_y + input.height * input.resolution);
  auto output = mrpt::maps::COccupancyGridMap2D::Create(
      static_cast<float>(input.origin_x), x1,
      static_cast<float>(input.origin_y), y1,
      static_cast<float>(input.resolution));
  output->fill(0.5f);
  for (int y = 0; y < input.height; ++y) {
    for (int x = 0; x < input.width; ++x) {
      const int16_t value = input.values[x + y * input.width];
      // ROS OccupancyGrid: -1 unknown, 0..49 free, >=50 occupied.
      // MRPT COccupancyGridMap2D: free=1, occupied=0, unknown=.5.
      const float probability = value < 0 ? 0.5f : (value >= 50 ? 0.0f : 1.0f);
      output->setCell(x, y, probability);
    }
  }
  return output;
}

PyObject* align_maps(PyObject*, PyObject* args)
{
  try {
    if (!PyTuple_Check(args) || PyTuple_Size(args) != 15) {
      throw std::runtime_error(
          "align_maps expects src,dst,src_width,src_height,dst_width,dst_height,resolution,"
          "src_x,src_y,src_yaw,dst_x,dst_y,dst_yaw,max_kld,max_modes");
    }
    PyObject* source_object = PyTuple_GetItem(args, 0);
    PyObject* target_object = PyTuple_GetItem(args, 1);
    if (!PyBytes_Check(source_object) || !PyBytes_Check(target_object)) {
      throw std::runtime_error("source and target values must be bytes");
    }
    const int source_width = integer_argument<int>(args, 2, "source_width");
    const int source_height = integer_argument<int>(args, 3, "source_height");
    const int target_width = integer_argument<int>(args, 4, "target_width");
    const int target_height = integer_argument<int>(args, 5, "target_height");
    const double resolution = floating_argument(args, 6, "resolution");
    const double source_x = floating_argument(args, 7, "source_x");
    const double source_y = floating_argument(args, 8, "source_y");
    const double source_yaw = floating_argument(args, 9, "source_yaw");
    const double target_x = floating_argument(args, 10, "target_x");
    const double target_y = floating_argument(args, 11, "target_y");
    const double target_yaw = floating_argument(args, 12, "target_yaw");
    const double max_kld = floating_argument(args, 13, "max_kld");
    const int max_modes = integer_argument<int>(args, 14, "max_modes");
    if (source_width <= 0 || source_height <= 0 || target_width <= 0 ||
        target_height <= 0 || !std::isfinite(resolution) ||
        resolution <= 0.0 || max_modes <= 0 || !std::isfinite(max_kld) ||
        max_kld < 0.0) {
      throw std::runtime_error("invalid map/alignment parameters");
    }
    // The current project crop protocol carries this metadata and the
    // official map is axis-aligned.  Refuse an unsupported rotation rather
    // than silently resampling or discarding map geometry.
    if (std::abs(source_yaw) > 1e-6 || std::abs(target_yaw) > 1e-6) {
      throw std::runtime_error(
          "MRPT grid backend requires axis-aligned crop origins (origin_yaw=0)");
    }
    const size_t expected_source = static_cast<size_t>(source_width) *
                                   static_cast<size_t>(source_height) *
                                   sizeof(int16_t);
    const size_t expected_target = static_cast<size_t>(target_width) *
                                   static_cast<size_t>(target_height) *
                                   sizeof(int16_t);
    if (static_cast<size_t>(PyBytes_Size(source_object)) != expected_source ||
        static_cast<size_t>(PyBytes_Size(target_object)) != expected_target) {
      throw std::runtime_error("map byte length does not match dimensions");
    }
    const MapSpec source{
        source_width, source_height, resolution, source_x, source_y, source_yaw,
        reinterpret_cast<const int16_t*>(PyBytes_AsString(source_object))};
    const MapSpec target{
        target_width, target_height, resolution, target_x, target_y, target_yaw,
        reinterpret_cast<const int16_t*>(PyBytes_AsString(target_object))};
    auto source_map = make_map(source);
    auto target_map = make_map(target);

    mrpt::slam::CGridMapAligner aligner;
    aligner.options.methodSelection =
        mrpt::slam::CGridMapAligner::amModifiedRANSAC;
    aligner.options.maxKLd_for_merge = max_kld;
    aligner.setMinLoggingLevel(mrpt::system::LVL_WARN);
    mrpt::poses::CPosePDFGaussian initial(
        mrpt::poses::CPose2D(0.0, 0.0, 0.0));
    auto pdf = aligner.AlignPDF(source_map.get(), target_map.get(), initial);
    auto sog = std::dynamic_pointer_cast<mrpt::poses::CPosePDFSOG>(pdf);
    if (!sog) return PyList_New(0);

    struct Mode {
      mrpt::poses::CPosePDFSOG::TGaussianMode value;
    };
    std::vector<Mode> modes;
    modes.reserve(sog->size());
    for (const auto& value : sog->getSOGModes()) modes.push_back({value});
    std::sort(modes.begin(), modes.end(), [](const Mode& left, const Mode& right) {
      return left.value.log_w > right.value.log_w;
    });
    if (static_cast<int>(modes.size()) > max_modes) {
      modes.resize(static_cast<size_t>(max_modes));
    }

    PyObject* output = PyList_New(static_cast<Py_ssize_t>(modes.size()));
    if (!output) return nullptr;
    for (size_t i = 0; i < modes.size(); ++i) {
      const auto& value = modes[i].value;
      // MRPT returns pose(m2 in m1).  The project canonical convention is
      // p_target = T_source_to_target p_source, hence exact inversion here.
      const auto canonical = inverse_pose(value.mean);
      PyObject* covariance = PyTuple_New(9);
      if (!covariance) {
        Py_DECREF(output);
        return nullptr;
      }
      for (int j = 0; j < 9; ++j) {
        PyTuple_SET_ITEM(
            covariance, j, PyFloat_FromDouble(value.cov(j / 3, j % 3)));
      }
      PyObject* item = Py_BuildValue(
          "(ddddOi)", canonical.x(), canonical.y(), canonical.phi(),
          value.log_w, covariance, static_cast<int>(i));
      Py_DECREF(covariance);
      if (!item) {
        Py_DECREF(output);
        return nullptr;
      }
      PyList_SET_ITEM(output, static_cast<Py_ssize_t>(i), item);
    }
    return output;
  } catch (const std::exception& error) {
    PyErr_SetString(PyExc_RuntimeError, error.what());
    return nullptr;
  }
}

PyMethodDef methods[] = {
    {"align_maps", align_maps, METH_VARARGS,
     "Return bounded canonical MRPT modified-RANSAC modes."},
    {nullptr, nullptr, 0, nullptr},
};

PyModuleDef module = {
    PyModuleDef_HEAD_INIT, "_mrpt_registration", nullptr, -1, methods,
    nullptr, nullptr, nullptr, nullptr};

}  // namespace

PyMODINIT_FUNC PyInit__mrpt_registration() { return PyModule_Create(&module); }
