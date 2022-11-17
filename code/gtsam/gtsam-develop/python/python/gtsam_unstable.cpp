/**
 * @file  gtsam.cpp
 * @brief   The auto-generated wrapper C++ source code.
 * @author  Duy-Nguyen Ta, Fan Jiang, Matthew Sklar
 * @date  Aug. 18, 2020
 *
 * ** THIS FILE IS AUTO-GENERATED, DO NOT MODIFY! **
 */

// Include relevant boost libraries required by GTSAM
#include <boost/shared_ptr.hpp>

#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/iostream.h>
#include "gtsam/base/serialization.h"
#include "gtsam/base/utilities.h"  // for RedirectCout.

// These are the included headers listed in `gtsam_unstable.i`
#include "gtsam_unstable/base/Dummy.h"
#include "gtsam_unstable/dynamics/PoseRTV.h"
#include "gtsam_unstable/geometry/Pose3Upright.h"
#include "gtsam_unstable/geometry/BearingS2.h"
#include "gtsam_unstable/geometry/SimWall2D.h"
#include "gtsam_unstable/geometry/SimPolygon2D.h"
#include "gtsam/nonlinear/PriorFactor.h"
#include "gtsam/slam/BetweenFactor.h"
#include "gtsam_unstable/slam/BetweenFactorEM.h"
#include "gtsam_unstable/slam/TransformBtwRobotsUnaryFactorEM.h"
#include "gtsam_unstable/slam/TransformBtwRobotsUnaryFactor.h"
#include "gtsam_unstable/slam/SmartRangeFactor.h"
#include "gtsam/sam/RangeFactor.h"
#include "gtsam_unstable/geometry/Event.h"
#include "gtsam_unstable/slam/TOAFactor.h"
#include "gtsam/nonlinear/NonlinearEquality.h"
#include "gtsam_unstable/dynamics/IMUFactor.h"
#include "gtsam_unstable/dynamics/FullIMUFactor.h"
#include "gtsam_unstable/dynamics/DynamicsPriors.h"
#include "gtsam_unstable/dynamics/VelocityConstraint3.h"
#include "gtsam_unstable/dynamics/Pendulum.h"
#include "gtsam_unstable/dynamics/Pendulum.h"
#include "gtsam_unstable/dynamics/SimpleHelicopter.h"
#include "gtsam_unstable/nonlinear/FixedLagSmoother.h"
#include "gtsam_unstable/nonlinear/FixedLagSmoother.h"
#include "gtsam_unstable/nonlinear/BatchFixedLagSmoother.h"
#include "gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h"
#include "gtsam_unstable/nonlinear/ConcurrentFilteringAndSmoothing.h"
#include "gtsam_unstable/nonlinear/ConcurrentBatchFilter.h"
#include "gtsam_unstable/nonlinear/ConcurrentBatchSmoother.h"
#include "gtsam_unstable/slam/RelativeElevationFactor.h"
#include "gtsam_unstable/slam/DummyFactor.h"
#include "gtsam_unstable/slam/InvDepthFactorVariant1.h"
#include "gtsam_unstable/slam/InvDepthFactorVariant2.h"
#include "gtsam_unstable/slam/InvDepthFactorVariant3.h"
#include "gtsam_unstable/slam/Mechanization_bRn2.h"
#include "gtsam_unstable/slam/AHRS.h"
#include "gtsam_unstable/slam/TSAMFactors.h"
#include "gtsam/geometry/Cal3DS2.h"
#include "gtsam_unstable/slam/ProjectionFactorPPP.h"
#include "gtsam_unstable/slam/ProjectionFactorPPPC.h"
#include "gtsam_unstable/slam/ProjectionFactorRollingShutter.h"

#include <boost/serialization/export.hpp>

BOOST_CLASS_EXPORT(gtsam::PoseRTV)
BOOST_CLASS_EXPORT(gtsam::Pose3Upright)
BOOST_CLASS_EXPORT(gtsam::BearingS2)
BOOST_CLASS_EXPORT(gtsam::PriorFactor<gtsam::PoseRTV>)
BOOST_CLASS_EXPORT(gtsam::BetweenFactor<gtsam::PoseRTV>)
BOOST_CLASS_EXPORT(gtsam::BetweenFactorEM<gtsam::Pose2>)
BOOST_CLASS_EXPORT(gtsam::TransformBtwRobotsUnaryFactorEM<gtsam::Pose2>)
BOOST_CLASS_EXPORT(gtsam::TransformBtwRobotsUnaryFactor<gtsam::Pose2>)
BOOST_CLASS_EXPORT(gtsam::NonlinearEquality<gtsam::PoseRTV>)
BOOST_CLASS_EXPORT(gtsam::ProjectionFactorRollingShutter)
typedef gtsam::RangeFactor<gtsam::PoseRTV, gtsam::PoseRTV> gtsamRangeFactorgtsamPoseRTVgtsamPoseRTV;
BOOST_CLASS_EXPORT(gtsamRangeFactorgtsamPoseRTVgtsamPoseRTV)
typedef gtsam::ProjectionFactorPPP<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> gtsamProjectionFactorPPPgtsamPose3gtsamPoint3gtsamCal3_S2;
BOOST_CLASS_EXPORT(gtsamProjectionFactorPPPgtsamPose3gtsamPoint3gtsamCal3_S2)
typedef gtsam::ProjectionFactorPPP<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2> gtsamProjectionFactorPPPgtsamPose3gtsamPoint3gtsamCal3DS2;
BOOST_CLASS_EXPORT(gtsamProjectionFactorPPPgtsamPose3gtsamPoint3gtsamCal3DS2)
typedef gtsam::ProjectionFactorPPPC<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> gtsamProjectionFactorPPPCgtsamPose3gtsamPoint3gtsamCal3_S2;
BOOST_CLASS_EXPORT(gtsamProjectionFactorPPPCgtsamPose3gtsamPoint3gtsamCal3_S2)
typedef gtsam::ProjectionFactorPPPC<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2> gtsamProjectionFactorPPPCgtsamPose3gtsamPoint3gtsamCal3DS2;
BOOST_CLASS_EXPORT(gtsamProjectionFactorPPPCgtsamPose3gtsamPoint3gtsamCal3DS2)


PYBIND11_DECLARE_HOLDER_TYPE(TYPE_PLACEHOLDER_DONOTUSE, boost::shared_ptr<TYPE_PLACEHOLDER_DONOTUSE>);

#include "python/gtsam_unstable/preamble.h"

using namespace std;

namespace py = pybind11;

PYBIND11_MODULE(gtsam_unstable, m_) {
    m_.doc() = "pybind11 wrapper of gtsam_unstable";

    // Note here we need to import the dependent library
    py::module::import("gtsam");


    py::class_<gtsam::Dummy, boost::shared_ptr<gtsam::Dummy>>(m_, "Dummy")
        .def(py::init<>())
        .def("print",[](gtsam::Dummy* self, string s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s"))
        .def("__repr__",
                    [](const gtsam::Dummy& self, string s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s"))
        .def("dummyTwoVar",[](gtsam::Dummy* self, unsigned char a){return self->dummyTwoVar(a);}, py::arg("a"));

    py::class_<gtsam::PoseRTV, boost::shared_ptr<gtsam::PoseRTV>>(m_, "PoseRTV")
        .def(py::init<>())
        .def(py::init<const gtsam::Vector&>(), py::arg("rtv"))
        .def(py::init<const gtsam::Point3&, const gtsam::Rot3&, const gtsam::Vector&>(), py::arg("pt"), py::arg("rot"), py::arg("vel"))
        .def(py::init<const gtsam::Rot3&, const gtsam::Point3&, const gtsam::Vector&>(), py::arg("rot"), py::arg("pt"), py::arg("vel"))
        .def(py::init<const gtsam::Pose3&, const gtsam::Vector&>(), py::arg("pose"), py::arg("vel"))
        .def(py::init<const gtsam::Pose3&>(), py::arg("pose"))
        .def(py::init<double, double, double, double, double, double, double, double, double>(), py::arg("roll"), py::arg("pitch"), py::arg("yaw"), py::arg("x"), py::arg("y"), py::arg("z"), py::arg("vx"), py::arg("vy"), py::arg("vz"))
        .def("equals",[](gtsam::PoseRTV* self, const gtsam::PoseRTV& other, double tol){return self->equals(other, tol);}, py::arg("other"), py::arg("tol"))
        .def("print",[](gtsam::PoseRTV* self, string s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s"))
        .def("__repr__",
                    [](const gtsam::PoseRTV& self, string s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s"))
        .def("translation",[](gtsam::PoseRTV* self){return self->translation();})
        .def("rotation",[](gtsam::PoseRTV* self){return self->rotation();})
        .def("velocity",[](gtsam::PoseRTV* self){return self->velocity();})
        .def("pose",[](gtsam::PoseRTV* self){return self->pose();})
        .def("vector",[](gtsam::PoseRTV* self){return self->vector();})
        .def("translationVec",[](gtsam::PoseRTV* self){return self->translationVec();})
        .def("velocityVec",[](gtsam::PoseRTV* self){return self->velocityVec();})
        .def("dim",[](gtsam::PoseRTV* self){return self->dim();})
        .def("retract",[](gtsam::PoseRTV* self, const gtsam::Vector& v){return self->retract(v);}, py::arg("v"))
        .def("localCoordinates",[](gtsam::PoseRTV* self, const gtsam::PoseRTV& p){return self->localCoordinates(p);}, py::arg("p"))
        .def("inverse",[](gtsam::PoseRTV* self){return self->inverse();})
        .def("compose",[](gtsam::PoseRTV* self, const gtsam::PoseRTV& p){return self->compose(p);}, py::arg("p"))
        .def("between",[](gtsam::PoseRTV* self, const gtsam::PoseRTV& p){return self->between(p);}, py::arg("p"))
        .def("range",[](gtsam::PoseRTV* self, const gtsam::PoseRTV& other){return self->range(other);}, py::arg("other"))
        .def("transformed_from",[](gtsam::PoseRTV* self, const gtsam::Pose3& trans){return self->transformed_from(trans);}, py::arg("trans"))
        .def("planarDynamics",[](gtsam::PoseRTV* self, double vel_rate, double heading_rate, double max_accel, double dt){return self->planarDynamics(vel_rate, heading_rate, max_accel, dt);}, py::arg("vel_rate"), py::arg("heading_rate"), py::arg("max_accel"), py::arg("dt"))
        .def("flyingDynamics",[](gtsam::PoseRTV* self, double pitch_rate, double heading_rate, double lift_control, double dt){return self->flyingDynamics(pitch_rate, heading_rate, lift_control, dt);}, py::arg("pitch_rate"), py::arg("heading_rate"), py::arg("lift_control"), py::arg("dt"))
        .def("generalDynamics",[](gtsam::PoseRTV* self, const gtsam::Vector& accel, const gtsam::Vector& gyro, double dt){return self->generalDynamics(accel, gyro, dt);}, py::arg("accel"), py::arg("gyro"), py::arg("dt"))
        .def("imuPrediction",[](gtsam::PoseRTV* self, const gtsam::PoseRTV& x2, double dt){return self->imuPrediction(x2, dt);}, py::arg("x2"), py::arg("dt"))
        .def("translationIntegration",[](gtsam::PoseRTV* self, const gtsam::PoseRTV& x2, double dt){return self->translationIntegration(x2, dt);}, py::arg("x2"), py::arg("dt"))
        .def("translationIntegrationVec",[](gtsam::PoseRTV* self, const gtsam::PoseRTV& x2, double dt){return self->translationIntegrationVec(x2, dt);}, py::arg("x2"), py::arg("dt"))
        .def("serialize", [](gtsam::PoseRTV* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::PoseRTV* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::PoseRTV &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::PoseRTV obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def_static("Dim",[](){return gtsam::PoseRTV::Dim();})
        .def_static("Expmap",[](const gtsam::Vector& v){return gtsam::PoseRTV::Expmap(v);}, py::arg("v"))
        .def_static("Logmap",[](const gtsam::PoseRTV& p){return gtsam::PoseRTV::Logmap(p);}, py::arg("p"));

    py::class_<gtsam::Pose3Upright, boost::shared_ptr<gtsam::Pose3Upright>>(m_, "Pose3Upright")
        .def(py::init<>())
        .def(py::init<const gtsam::Pose3Upright&>(), py::arg("other"))
        .def(py::init<const gtsam::Rot2&, const gtsam::Point3&>(), py::arg("bearing"), py::arg("t"))
        .def(py::init<double, double, double, double>(), py::arg("x"), py::arg("y"), py::arg("z"), py::arg("theta"))
        .def(py::init<const gtsam::Pose2&, double>(), py::arg("pose"), py::arg("z"))
        .def("print",[](gtsam::Pose3Upright* self, string s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s"))
        .def("__repr__",
                    [](const gtsam::Pose3Upright& self, string s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s"))
        .def("equals",[](gtsam::Pose3Upright* self, const gtsam::Pose3Upright& pose, double tol){return self->equals(pose, tol);}, py::arg("pose"), py::arg("tol"))
        .def("x",[](gtsam::Pose3Upright* self){return self->x();})
        .def("y",[](gtsam::Pose3Upright* self){return self->y();})
        .def("z",[](gtsam::Pose3Upright* self){return self->z();})
        .def("theta",[](gtsam::Pose3Upright* self){return self->theta();})
        .def("translation2",[](gtsam::Pose3Upright* self){return self->translation2();})
        .def("translation",[](gtsam::Pose3Upright* self){return self->translation();})
        .def("rotation2",[](gtsam::Pose3Upright* self){return self->rotation2();})
        .def("rotation",[](gtsam::Pose3Upright* self){return self->rotation();})
        .def("pose2",[](gtsam::Pose3Upright* self){return self->pose2();})
        .def("pose",[](gtsam::Pose3Upright* self){return self->pose();})
        .def("dim",[](gtsam::Pose3Upright* self){return self->dim();})
        .def("retract",[](gtsam::Pose3Upright* self, const gtsam::Vector& v){return self->retract(v);}, py::arg("v"))
        .def("localCoordinates",[](gtsam::Pose3Upright* self, const gtsam::Pose3Upright& p2){return self->localCoordinates(p2);}, py::arg("p2"))
        .def("inverse",[](gtsam::Pose3Upright* self){return self->inverse();})
        .def("compose",[](gtsam::Pose3Upright* self, const gtsam::Pose3Upright& p2){return self->compose(p2);}, py::arg("p2"))
        .def("between",[](gtsam::Pose3Upright* self, const gtsam::Pose3Upright& p2){return self->between(p2);}, py::arg("p2"))
        .def("serialize", [](gtsam::Pose3Upright* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::Pose3Upright* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::Pose3Upright &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::Pose3Upright obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def_static("Identity",[](){return gtsam::Pose3Upright::Identity();})
        .def_static("Expmap",[](const gtsam::Vector& xi){return gtsam::Pose3Upright::Expmap(xi);}, py::arg("xi"))
        .def_static("Logmap",[](const gtsam::Pose3Upright& p){return gtsam::Pose3Upright::Logmap(p);}, py::arg("p"));

    py::class_<gtsam::BearingS2, boost::shared_ptr<gtsam::BearingS2>>(m_, "BearingS2")
        .def(py::init<>())
        .def(py::init<double, double>(), py::arg("azimuth_double"), py::arg("elevation_double"))
        .def(py::init<const gtsam::Rot2&, const gtsam::Rot2&>(), py::arg("azimuth"), py::arg("elevation"))
        .def("azimuth",[](gtsam::BearingS2* self){return self->azimuth();})
        .def("elevation",[](gtsam::BearingS2* self){return self->elevation();})
        .def("print",[](gtsam::BearingS2* self, string s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s"))
        .def("__repr__",
                    [](const gtsam::BearingS2& self, string s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s"))
        .def("equals",[](gtsam::BearingS2* self, const gtsam::BearingS2& x, double tol){return self->equals(x, tol);}, py::arg("x"), py::arg("tol"))
        .def("dim",[](gtsam::BearingS2* self){return self->dim();})
        .def("retract",[](gtsam::BearingS2* self, const gtsam::Vector& v){return self->retract(v);}, py::arg("v"))
        .def("localCoordinates",[](gtsam::BearingS2* self, const gtsam::BearingS2& p2){return self->localCoordinates(p2);}, py::arg("p2"))
        .def("serialize", [](gtsam::BearingS2* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::BearingS2* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::BearingS2 &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::BearingS2 obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def_static("fromDownwardsObservation",[](const gtsam::Pose3& A, const gtsam::Point3& B){return gtsam::BearingS2::fromDownwardsObservation(A, B);}, py::arg("A"), py::arg("B"))
        .def_static("fromForwardObservation",[](const gtsam::Pose3& A, const gtsam::Point3& B){return gtsam::BearingS2::fromForwardObservation(A, B);}, py::arg("A"), py::arg("B"));

    py::class_<gtsam::SimWall2D, boost::shared_ptr<gtsam::SimWall2D>>(m_, "SimWall2D")
        .def(py::init<>())
        .def(py::init<const gtsam::Point2&, const gtsam::Point2&>(), py::arg("a"), py::arg("b"))
        .def(py::init<double, double, double, double>(), py::arg("ax"), py::arg("ay"), py::arg("bx"), py::arg("by"))
        .def("print",[](gtsam::SimWall2D* self, string s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s"))
        .def("__repr__",
                    [](const gtsam::SimWall2D& self, string s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s"))
        .def("equals",[](gtsam::SimWall2D* self, const gtsam::SimWall2D& other, double tol){return self->equals(other, tol);}, py::arg("other"), py::arg("tol"))
        .def("a",[](gtsam::SimWall2D* self){return self->a();})
        .def("b",[](gtsam::SimWall2D* self){return self->b();})
        .def("scale",[](gtsam::SimWall2D* self, double s){return self->scale(s);}, py::arg("s"))
        .def("length",[](gtsam::SimWall2D* self){return self->length();})
        .def("midpoint",[](gtsam::SimWall2D* self){return self->midpoint();})
        .def("intersects",[](gtsam::SimWall2D* self, const gtsam::SimWall2D& wall){return self->intersects(wall);}, py::arg("wall"))
        .def("norm",[](gtsam::SimWall2D* self){return self->norm();})
        .def("reflection",[](gtsam::SimWall2D* self, const gtsam::Point2& init, const gtsam::Point2& intersection){return self->reflection(init, intersection);}, py::arg("init"), py::arg("intersection"));

    py::class_<gtsam::SimPolygon2D, boost::shared_ptr<gtsam::SimPolygon2D>>(m_, "SimPolygon2D")
        .def("landmark",[](gtsam::SimPolygon2D* self, size_t i){return self->landmark(i);}, py::arg("i"))
        .def("size",[](gtsam::SimPolygon2D* self){return self->size();})
        .def("vertices",[](gtsam::SimPolygon2D* self){return self->vertices();})
        .def("equals",[](gtsam::SimPolygon2D* self, const gtsam::SimPolygon2D& p, double tol){return self->equals(p, tol);}, py::arg("p"), py::arg("tol"))
        .def("print",[](gtsam::SimPolygon2D* self, string s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s"))
        .def("__repr__",
                    [](const gtsam::SimPolygon2D& self, string s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s"))
        .def("walls",[](gtsam::SimPolygon2D* self){return self->walls();})
        .def("contains",[](gtsam::SimPolygon2D* self, const gtsam::Point2& p){return self->contains(p);}, py::arg("p"))
        .def("overlaps",[](gtsam::SimPolygon2D* self, const gtsam::SimPolygon2D& p){return self->overlaps(p);}, py::arg("p"))
        .def_static("seedGenerator",[](size_t seed){ gtsam::SimPolygon2D::seedGenerator(seed);}, py::arg("seed"))
        .def_static("createTriangle",[](const gtsam::Point2& pA, const gtsam::Point2& pB, const gtsam::Point2& pC){return gtsam::SimPolygon2D::createTriangle(pA, pB, pC);}, py::arg("pA"), py::arg("pB"), py::arg("pC"))
        .def_static("createRectangle",[](const gtsam::Point2& p, double height, double width){return gtsam::SimPolygon2D::createRectangle(p, height, width);}, py::arg("p"), py::arg("height"), py::arg("width"))
        .def_static("randomTriangle",[](double side_len, double mean_side_len, double sigma_side_len, double min_vertex_dist, double min_side_len, const gtsam::SimPolygon2DVector& existing_polys){return gtsam::SimPolygon2D::randomTriangle(side_len, mean_side_len, sigma_side_len, min_vertex_dist, min_side_len, existing_polys);}, py::arg("side_len"), py::arg("mean_side_len"), py::arg("sigma_side_len"), py::arg("min_vertex_dist"), py::arg("min_side_len"), py::arg("existing_polys"))
        .def_static("randomRectangle",[](double side_len, double mean_side_len, double sigma_side_len, double min_vertex_dist, double min_side_len, const gtsam::SimPolygon2DVector& existing_polys){return gtsam::SimPolygon2D::randomRectangle(side_len, mean_side_len, sigma_side_len, min_vertex_dist, min_side_len, existing_polys);}, py::arg("side_len"), py::arg("mean_side_len"), py::arg("sigma_side_len"), py::arg("min_vertex_dist"), py::arg("min_side_len"), py::arg("existing_polys"))
        .def_static("anyContains",[](const gtsam::Point2& p, const gtsam::SimPolygon2DVector& obstacles){return gtsam::SimPolygon2D::anyContains(p, obstacles);}, py::arg("p"), py::arg("obstacles"))
        .def_static("anyOverlaps",[](const gtsam::SimPolygon2D& p, const gtsam::SimPolygon2DVector& obstacles){return gtsam::SimPolygon2D::anyOverlaps(p, obstacles);}, py::arg("p"), py::arg("obstacles"))
        .def_static("insideBox",[](double s, const gtsam::Point2& p){return gtsam::SimPolygon2D::insideBox(s, p);}, py::arg("s"), py::arg("p"))
        .def_static("nearExisting",[](const gtsam::Point2Vector& S, const gtsam::Point2& p, double threshold){return gtsam::SimPolygon2D::nearExisting(S, p, threshold);}, py::arg("S"), py::arg("p"), py::arg("threshold"))
        .def_static("randomPoint2",[](double s){return gtsam::SimPolygon2D::randomPoint2(s);}, py::arg("s"))
        .def_static("randomAngle",[](){return gtsam::SimPolygon2D::randomAngle();})
        .def_static("randomDistance",[](double mu, double sigma){return gtsam::SimPolygon2D::randomDistance(mu, sigma);}, py::arg("mu"), py::arg("sigma"))
        .def_static("randomDistance",[](double mu, double sigma, double min_dist){return gtsam::SimPolygon2D::randomDistance(mu, sigma, min_dist);}, py::arg("mu"), py::arg("sigma"), py::arg("min_dist"))
        .def_static("randomBoundedPoint2",[](double boundary_size, const gtsam::Point2Vector& landmarks, double min_landmark_dist){return gtsam::SimPolygon2D::randomBoundedPoint2(boundary_size, landmarks, min_landmark_dist);}, py::arg("boundary_size"), py::arg("landmarks"), py::arg("min_landmark_dist"))
        .def_static("randomBoundedPoint2",[](double boundary_size, const gtsam::Point2Vector& landmarks, const gtsam::SimPolygon2DVector& obstacles, double min_landmark_dist){return gtsam::SimPolygon2D::randomBoundedPoint2(boundary_size, landmarks, obstacles, min_landmark_dist);}, py::arg("boundary_size"), py::arg("landmarks"), py::arg("obstacles"), py::arg("min_landmark_dist"))
        .def_static("randomBoundedPoint2",[](double boundary_size, const gtsam::SimPolygon2DVector& obstacles){return gtsam::SimPolygon2D::randomBoundedPoint2(boundary_size, obstacles);}, py::arg("boundary_size"), py::arg("obstacles"))
        .def_static("randomBoundedPoint2",[](const gtsam::Point2& LL_corner, const gtsam::Point2& UR_corner, const gtsam::Point2Vector& landmarks, const gtsam::SimPolygon2DVector& obstacles, double min_landmark_dist){return gtsam::SimPolygon2D::randomBoundedPoint2(LL_corner, UR_corner, landmarks, obstacles, min_landmark_dist);}, py::arg("LL_corner"), py::arg("UR_corner"), py::arg("landmarks"), py::arg("obstacles"), py::arg("min_landmark_dist"))
        .def_static("randomFreePose",[](double boundary_size, const gtsam::SimPolygon2DVector& obstacles){return gtsam::SimPolygon2D::randomFreePose(boundary_size, obstacles);}, py::arg("boundary_size"), py::arg("obstacles"));

    py::class_<gtsam::SimWall2DVector, boost::shared_ptr<gtsam::SimWall2DVector>>(m_, "SimWall2DVector")
        .def("size",[](gtsam::SimWall2DVector* self){return self->size();})
        .def("max_size",[](gtsam::SimWall2DVector* self){return self->max_size();})
        .def("resize",[](gtsam::SimWall2DVector* self, size_t sz){ self->resize(sz);}, py::arg("sz"))
        .def("capacity",[](gtsam::SimWall2DVector* self){return self->capacity();})
        .def("empty",[](gtsam::SimWall2DVector* self){return self->empty();})
        .def("reserve",[](gtsam::SimWall2DVector* self, size_t n){ self->reserve(n);}, py::arg("n"))
        .def("at",[](gtsam::SimWall2DVector* self, size_t n){return self->at(n);}, py::arg("n"))
        .def("front",[](gtsam::SimWall2DVector* self){return self->front();})
        .def("back",[](gtsam::SimWall2DVector* self){return self->back();})
        .def("assign",[](gtsam::SimWall2DVector* self, size_t n, const gtsam::SimWall2D& u){ self->assign(n, u);}, py::arg("n"), py::arg("u"))
        .def("push_back",[](gtsam::SimWall2DVector* self, const gtsam::SimWall2D& x){ self->push_back(x);}, py::arg("x"))
        .def("pop_back",[](gtsam::SimWall2DVector* self){ self->pop_back();});

    py::class_<gtsam::SimPolygon2DVector, boost::shared_ptr<gtsam::SimPolygon2DVector>>(m_, "SimPolygon2DVector")
        .def("size",[](gtsam::SimPolygon2DVector* self){return self->size();})
        .def("max_size",[](gtsam::SimPolygon2DVector* self){return self->max_size();})
        .def("resize",[](gtsam::SimPolygon2DVector* self, size_t sz){ self->resize(sz);}, py::arg("sz"))
        .def("capacity",[](gtsam::SimPolygon2DVector* self){return self->capacity();})
        .def("empty",[](gtsam::SimPolygon2DVector* self){return self->empty();})
        .def("reserve",[](gtsam::SimPolygon2DVector* self, size_t n){ self->reserve(n);}, py::arg("n"))
        .def("at",[](gtsam::SimPolygon2DVector* self, size_t n){return self->at(n);}, py::arg("n"))
        .def("front",[](gtsam::SimPolygon2DVector* self){return self->front();})
        .def("back",[](gtsam::SimPolygon2DVector* self){return self->back();})
        .def("assign",[](gtsam::SimPolygon2DVector* self, size_t n, const gtsam::SimPolygon2D& u){ self->assign(n, u);}, py::arg("n"), py::arg("u"))
        .def("push_back",[](gtsam::SimPolygon2DVector* self, const gtsam::SimPolygon2D& x){ self->push_back(x);}, py::arg("x"))
        .def("pop_back",[](gtsam::SimPolygon2DVector* self){ self->pop_back();});

    py::class_<gtsam::PriorFactor<gtsam::PoseRTV>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::PriorFactor<gtsam::PoseRTV>>>(m_, "PriorFactorPoseRTV")
        .def(py::init<size_t, const gtsam::PoseRTV&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("serialize", [](gtsam::PriorFactor<gtsam::PoseRTV>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::PriorFactor<gtsam::PoseRTV>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::PriorFactor<gtsam::PoseRTV> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::PriorFactor<gtsam::PoseRTV> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::BetweenFactor<gtsam::PoseRTV>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::BetweenFactor<gtsam::PoseRTV>>>(m_, "BetweenFactorPoseRTV")
        .def(py::init<size_t, size_t, const gtsam::PoseRTV&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key1"), py::arg("key2"), py::arg("relativePose"), py::arg("noiseModel"))
        .def("serialize", [](gtsam::BetweenFactor<gtsam::PoseRTV>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::BetweenFactor<gtsam::PoseRTV>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::BetweenFactor<gtsam::PoseRTV> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::BetweenFactor<gtsam::PoseRTV> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::BetweenFactorEM<gtsam::Pose2>, gtsam::NonlinearFactor, boost::shared_ptr<gtsam::BetweenFactorEM<gtsam::Pose2>>>(m_, "BetweenFactorEMPose2")
        .def(py::init<size_t, size_t, const gtsam::Pose2&, const boost::shared_ptr<gtsam::noiseModel::Gaussian>, const boost::shared_ptr<gtsam::noiseModel::Gaussian>, double, double>(), py::arg("key1"), py::arg("key2"), py::arg("relativePose"), py::arg("model_inlier"), py::arg("model_outlier"), py::arg("prior_inlier"), py::arg("prior_outlier"))
        .def(py::init<size_t, size_t, const gtsam::Pose2&, const boost::shared_ptr<gtsam::noiseModel::Gaussian>, const boost::shared_ptr<gtsam::noiseModel::Gaussian>, double, double, bool>(), py::arg("key1"), py::arg("key2"), py::arg("relativePose"), py::arg("model_inlier"), py::arg("model_outlier"), py::arg("prior_inlier"), py::arg("prior_outlier"), py::arg("flag_bump_up_near_zero_probs"))
        .def("whitenedError",[](gtsam::BetweenFactorEM<gtsam::Pose2>* self, const gtsam::Values& x){return self->whitenedError(x);}, py::arg("x"))
        .def("unwhitenedError",[](gtsam::BetweenFactorEM<gtsam::Pose2>* self, const gtsam::Values& x){return self->unwhitenedError(x);}, py::arg("x"))
        .def("calcIndicatorProb",[](gtsam::BetweenFactorEM<gtsam::Pose2>* self, const gtsam::Values& x){return self->calcIndicatorProb(x);}, py::arg("x"))
        .def("measured",[](gtsam::BetweenFactorEM<gtsam::Pose2>* self){return self->measured();})
        .def("set_flag_bump_up_near_zero_probs",[](gtsam::BetweenFactorEM<gtsam::Pose2>* self, bool flag){ self->set_flag_bump_up_near_zero_probs(flag);}, py::arg("flag"))
        .def("get_flag_bump_up_near_zero_probs",[](gtsam::BetweenFactorEM<gtsam::Pose2>* self){return self->get_flag_bump_up_near_zero_probs();})
        .def("updateNoiseModels",[](gtsam::BetweenFactorEM<gtsam::Pose2>* self, const gtsam::Values& values, const gtsam::NonlinearFactorGraph& graph){ self->updateNoiseModels(values, graph);}, py::arg("values"), py::arg("graph"))
        .def("updateNoiseModels_givenCovs",[](gtsam::BetweenFactorEM<gtsam::Pose2>* self, const gtsam::Values& values, const gtsam::Matrix& cov1, const gtsam::Matrix& cov2, const gtsam::Matrix& cov12){ self->updateNoiseModels_givenCovs(values, cov1, cov2, cov12);}, py::arg("values"), py::arg("cov1"), py::arg("cov2"), py::arg("cov12"))
        .def("get_model_inlier_cov",[](gtsam::BetweenFactorEM<gtsam::Pose2>* self){return self->get_model_inlier_cov();})
        .def("get_model_outlier_cov",[](gtsam::BetweenFactorEM<gtsam::Pose2>* self){return self->get_model_outlier_cov();})
        .def("serialize", [](gtsam::BetweenFactorEM<gtsam::Pose2>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::BetweenFactorEM<gtsam::Pose2>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::BetweenFactorEM<gtsam::Pose2> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::BetweenFactorEM<gtsam::Pose2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::TransformBtwRobotsUnaryFactorEM<gtsam::Pose2>, gtsam::NonlinearFactor, boost::shared_ptr<gtsam::TransformBtwRobotsUnaryFactorEM<gtsam::Pose2>>>(m_, "TransformBtwRobotsUnaryFactorEMPose2")
        .def(py::init<size_t, const gtsam::Pose2&, size_t, size_t, const gtsam::Values&, const gtsam::Values&, const boost::shared_ptr<gtsam::noiseModel::Gaussian>, const boost::shared_ptr<gtsam::noiseModel::Gaussian>, double, double>(), py::arg("key"), py::arg("relativePose"), py::arg("keyA"), py::arg("keyB"), py::arg("valA"), py::arg("valB"), py::arg("model_inlier"), py::arg("model_outlier"), py::arg("prior_inlier"), py::arg("prior_outlier"))
        .def(py::init<size_t, const gtsam::Pose2&, size_t, size_t, const gtsam::Values&, const gtsam::Values&, const boost::shared_ptr<gtsam::noiseModel::Gaussian>, const boost::shared_ptr<gtsam::noiseModel::Gaussian>, double, double, bool, bool>(), py::arg("key"), py::arg("relativePose"), py::arg("keyA"), py::arg("keyB"), py::arg("valA"), py::arg("valB"), py::arg("model_inlier"), py::arg("model_outlier"), py::arg("prior_inlier"), py::arg("prior_outlier"), py::arg("flag_bump_up_near_zero_probs"), py::arg("start_with_M_step"))
        .def("whitenedError",[](gtsam::TransformBtwRobotsUnaryFactorEM<gtsam::Pose2>* self, const gtsam::Values& x){return self->whitenedError(x);}, py::arg("x"))
        .def("unwhitenedError",[](gtsam::TransformBtwRobotsUnaryFactorEM<gtsam::Pose2>* self, const gtsam::Values& x){return self->unwhitenedError(x);}, py::arg("x"))
        .def("calcIndicatorProb",[](gtsam::TransformBtwRobotsUnaryFactorEM<gtsam::Pose2>* self, const gtsam::Values& x){return self->calcIndicatorProb(x);}, py::arg("x"))
        .def("setValAValB",[](gtsam::TransformBtwRobotsUnaryFactorEM<gtsam::Pose2>* self, const gtsam::Values valA, const gtsam::Values valB){ self->setValAValB(valA, valB);}, py::arg("valA"), py::arg("valB"))
        .def("updateNoiseModels",[](gtsam::TransformBtwRobotsUnaryFactorEM<gtsam::Pose2>* self, const gtsam::Values& values, const gtsam::NonlinearFactorGraph& graph){ self->updateNoiseModels(values, graph);}, py::arg("values"), py::arg("graph"))
        .def("updateNoiseModels_givenCovs",[](gtsam::TransformBtwRobotsUnaryFactorEM<gtsam::Pose2>* self, const gtsam::Values& values, const gtsam::Matrix& cov1, const gtsam::Matrix& cov2, const gtsam::Matrix& cov12){ self->updateNoiseModels_givenCovs(values, cov1, cov2, cov12);}, py::arg("values"), py::arg("cov1"), py::arg("cov2"), py::arg("cov12"))
        .def("get_model_inlier_cov",[](gtsam::TransformBtwRobotsUnaryFactorEM<gtsam::Pose2>* self){return self->get_model_inlier_cov();})
        .def("get_model_outlier_cov",[](gtsam::TransformBtwRobotsUnaryFactorEM<gtsam::Pose2>* self){return self->get_model_outlier_cov();})
        .def("serialize", [](gtsam::TransformBtwRobotsUnaryFactorEM<gtsam::Pose2>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::TransformBtwRobotsUnaryFactorEM<gtsam::Pose2>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::TransformBtwRobotsUnaryFactorEM<gtsam::Pose2> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::TransformBtwRobotsUnaryFactorEM<gtsam::Pose2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::TransformBtwRobotsUnaryFactor<gtsam::Pose2>, gtsam::NonlinearFactor, boost::shared_ptr<gtsam::TransformBtwRobotsUnaryFactor<gtsam::Pose2>>>(m_, "TransformBtwRobotsUnaryFactorPose2")
        .def(py::init<size_t, const gtsam::Pose2&, size_t, size_t, const gtsam::Values&, const gtsam::Values&, const boost::shared_ptr<gtsam::noiseModel::Gaussian>>(), py::arg("key"), py::arg("relativePose"), py::arg("keyA"), py::arg("keyB"), py::arg("valA"), py::arg("valB"), py::arg("model"))
        .def("whitenedError",[](gtsam::TransformBtwRobotsUnaryFactor<gtsam::Pose2>* self, const gtsam::Values& x){return self->whitenedError(x);}, py::arg("x"))
        .def("unwhitenedError",[](gtsam::TransformBtwRobotsUnaryFactor<gtsam::Pose2>* self, const gtsam::Values& x){return self->unwhitenedError(x);}, py::arg("x"))
        .def("setValAValB",[](gtsam::TransformBtwRobotsUnaryFactor<gtsam::Pose2>* self, const gtsam::Values valA, const gtsam::Values valB){ self->setValAValB(valA, valB);}, py::arg("valA"), py::arg("valB"))
        .def("serialize", [](gtsam::TransformBtwRobotsUnaryFactor<gtsam::Pose2>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::TransformBtwRobotsUnaryFactor<gtsam::Pose2>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::TransformBtwRobotsUnaryFactor<gtsam::Pose2> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::TransformBtwRobotsUnaryFactor<gtsam::Pose2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::SmartRangeFactor, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::SmartRangeFactor>>(m_, "SmartRangeFactor")
        .def(py::init<double>(), py::arg("s"))
        .def("addRange",[](gtsam::SmartRangeFactor* self, size_t key, double measuredRange){ self->addRange(key, measuredRange);}, py::arg("key"), py::arg("measuredRange"))
        .def("triangulate",[](gtsam::SmartRangeFactor* self, const gtsam::Values& x){return self->triangulate(x);}, py::arg("x"));

    py::class_<gtsam::Event, boost::shared_ptr<gtsam::Event>>(m_, "Event")
        .def(py::init<>())
        .def(py::init<double, const gtsam::Point3&>(), py::arg("t"), py::arg("p"))
        .def(py::init<double, double, double, double>(), py::arg("t"), py::arg("x"), py::arg("y"), py::arg("z"))
        .def("time",[](gtsam::Event* self){return self->time();})
        .def("location",[](gtsam::Event* self){return self->location();})
        .def("height",[](gtsam::Event* self){return self->height();})
        .def("print",[](gtsam::Event* self, string s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s"))
        .def("__repr__",
                    [](const gtsam::Event& self, string s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s"));

    py::class_<gtsam::TimeOfArrival, boost::shared_ptr<gtsam::TimeOfArrival>>(m_, "TimeOfArrival")
        .def(py::init<>())
        .def(py::init<double>(), py::arg("speed"))
        .def("measure",[](gtsam::TimeOfArrival* self, const gtsam::Event& event, const gtsam::Point3& sensor){return self->measure(event, sensor);}, py::arg("event"), py::arg("sensor"));

    py::class_<gtsam::TOAFactor, gtsam::NonlinearFactor, boost::shared_ptr<gtsam::TOAFactor>>(m_, "TOAFactor")
        .def(py::init<size_t, gtsam::Point3, double, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key1"), py::arg("sensor"), py::arg("measured"), py::arg("noiseModel"))
        .def_static("InsertEvent",[](size_t key, const gtsam::Event& event, boost::shared_ptr<gtsam::Values> values){ gtsam::TOAFactor::InsertEvent(key, event, values);}, py::arg("key"), py::arg("event"), py::arg("values"));

    py::class_<gtsam::NonlinearEquality<gtsam::PoseRTV>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::NonlinearEquality<gtsam::PoseRTV>>>(m_, "NonlinearEqualityPoseRTV")
        .def(py::init<size_t, const gtsam::PoseRTV&>(), py::arg("j"), py::arg("feasible"))
        .def(py::init<size_t, const gtsam::PoseRTV&, double>(), py::arg("j"), py::arg("feasible"), py::arg("error_gain"))
        .def("serialize", [](gtsam::NonlinearEquality<gtsam::PoseRTV>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::NonlinearEquality<gtsam::PoseRTV>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::NonlinearEquality<gtsam::PoseRTV> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::NonlinearEquality<gtsam::PoseRTV> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::IMUFactor<gtsam::PoseRTV>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::IMUFactor<gtsam::PoseRTV>>>(m_, "IMUFactorPoseRTV")
        .def(py::init<const gtsam::Vector&, const gtsam::Vector&, double, size_t, size_t, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("accel"), py::arg("gyro"), py::arg("dt"), py::arg("key1"), py::arg("key2"), py::arg("model"))
        .def(py::init<const gtsam::Vector&, double, size_t, size_t, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("imu_vector"), py::arg("dt"), py::arg("key1"), py::arg("key2"), py::arg("model"))
        .def("gyro",[](gtsam::IMUFactor<gtsam::PoseRTV>* self){return self->gyro();})
        .def("accel",[](gtsam::IMUFactor<gtsam::PoseRTV>* self){return self->accel();})
        .def("z",[](gtsam::IMUFactor<gtsam::PoseRTV>* self){return self->z();})
        .def("key1",[](gtsam::IMUFactor<gtsam::PoseRTV>* self){return self->key1();})
        .def("key2",[](gtsam::IMUFactor<gtsam::PoseRTV>* self){return self->key2();});

    py::class_<gtsam::FullIMUFactor<gtsam::PoseRTV>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::FullIMUFactor<gtsam::PoseRTV>>>(m_, "FullIMUFactorPoseRTV")
        .def(py::init<const gtsam::Vector&, const gtsam::Vector&, double, size_t, size_t, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("accel"), py::arg("gyro"), py::arg("dt"), py::arg("key1"), py::arg("key2"), py::arg("model"))
        .def(py::init<const gtsam::Vector&, double, size_t, size_t, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("imu"), py::arg("dt"), py::arg("key1"), py::arg("key2"), py::arg("model"))
        .def("gyro",[](gtsam::FullIMUFactor<gtsam::PoseRTV>* self){return self->gyro();})
        .def("accel",[](gtsam::FullIMUFactor<gtsam::PoseRTV>* self){return self->accel();})
        .def("z",[](gtsam::FullIMUFactor<gtsam::PoseRTV>* self){return self->z();})
        .def("key1",[](gtsam::FullIMUFactor<gtsam::PoseRTV>* self){return self->key1();})
        .def("key2",[](gtsam::FullIMUFactor<gtsam::PoseRTV>* self){return self->key2();});

    py::class_<gtsam::DHeightPrior, gtsam::NonlinearFactor, boost::shared_ptr<gtsam::DHeightPrior>>(m_, "DHeightPrior")
        .def(py::init<size_t, double, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key"), py::arg("height"), py::arg("model"));

    py::class_<gtsam::DRollPrior, gtsam::NonlinearFactor, boost::shared_ptr<gtsam::DRollPrior>>(m_, "DRollPrior")
        .def(py::init<size_t, double, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key"), py::arg("wx"), py::arg("model"))
        .def(py::init<size_t, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key"), py::arg("model"));

    py::class_<gtsam::VelocityPrior, gtsam::NonlinearFactor, boost::shared_ptr<gtsam::VelocityPrior>>(m_, "VelocityPrior")
        .def(py::init<size_t, const gtsam::Vector&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key"), py::arg("vel"), py::arg("model"));

    py::class_<gtsam::DGroundConstraint, gtsam::NonlinearFactor, boost::shared_ptr<gtsam::DGroundConstraint>>(m_, "DGroundConstraint")
        .def(py::init<size_t, double, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key"), py::arg("height"), py::arg("model"))
        .def(py::init<size_t, const gtsam::Vector&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key"), py::arg("constraint"), py::arg("model"));

    py::class_<gtsam::VelocityConstraint3, gtsam::NonlinearFactor, boost::shared_ptr<gtsam::VelocityConstraint3>>(m_, "VelocityConstraint3")
        .def(py::init<size_t, size_t, size_t, double>(), py::arg("key1"), py::arg("key2"), py::arg("velKey"), py::arg("dt"))
        .def("evaluateError",[](gtsam::VelocityConstraint3* self, const double& x1, const double& x2, const double& v){return self->evaluateError(x1, x2, v);}, py::arg("x1"), py::arg("x2"), py::arg("v"));

    py::class_<gtsam::PendulumFactor1, gtsam::NonlinearFactor, boost::shared_ptr<gtsam::PendulumFactor1>>(m_, "PendulumFactor1")
        .def(py::init<size_t, size_t, size_t, double>(), py::arg("k1"), py::arg("k"), py::arg("velKey"), py::arg("dt"))
        .def("evaluateError",[](gtsam::PendulumFactor1* self, const double& qk1, const double& qk, const double& v){return self->evaluateError(qk1, qk, v);}, py::arg("qk1"), py::arg("qk"), py::arg("v"));

    py::class_<gtsam::PendulumFactor2, gtsam::NonlinearFactor, boost::shared_ptr<gtsam::PendulumFactor2>>(m_, "PendulumFactor2")
        .def(py::init<size_t, size_t, size_t, double, double, double>(), py::arg("vk1"), py::arg("vk"), py::arg("qKey"), py::arg("dt"), py::arg("L"), py::arg("g"))
        .def("evaluateError",[](gtsam::PendulumFactor2* self, const double& vk1, const double& vk, const double& q){return self->evaluateError(vk1, vk, q);}, py::arg("vk1"), py::arg("vk"), py::arg("q"));

    py::class_<gtsam::PendulumFactorPk, gtsam::NonlinearFactor, boost::shared_ptr<gtsam::PendulumFactorPk>>(m_, "PendulumFactorPk")
        .def(py::init<size_t, size_t, size_t, double, double, double, double, double>(), py::arg("pk"), py::arg("qk"), py::arg("qk1"), py::arg("h"), py::arg("m"), py::arg("r"), py::arg("g"), py::arg("alpha"))
        .def("evaluateError",[](gtsam::PendulumFactorPk* self, const double& pk, const double& qk, const double& qk1){return self->evaluateError(pk, qk, qk1);}, py::arg("pk"), py::arg("qk"), py::arg("qk1"));

    py::class_<gtsam::PendulumFactorPk1, gtsam::NonlinearFactor, boost::shared_ptr<gtsam::PendulumFactorPk1>>(m_, "PendulumFactorPk1")
        .def(py::init<size_t, size_t, size_t, double, double, double, double, double>(), py::arg("pk1"), py::arg("qk"), py::arg("qk1"), py::arg("h"), py::arg("m"), py::arg("r"), py::arg("g"), py::arg("alpha"))
        .def("evaluateError",[](gtsam::PendulumFactorPk1* self, const double& pk1, const double& qk, const double& qk1){return self->evaluateError(pk1, qk, qk1);}, py::arg("pk1"), py::arg("qk"), py::arg("qk1"));

    py::class_<gtsam::Reconstruction, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::Reconstruction>>(m_, "Reconstruction")
        .def(py::init<size_t, size_t, size_t, double>(), py::arg("gKey1"), py::arg("gKey"), py::arg("xiKey"), py::arg("h"))
        .def("evaluateError",[](gtsam::Reconstruction* self, const gtsam::Pose3& gK1, const gtsam::Pose3& gK, const gtsam::Vector& xiK){return self->evaluateError(gK1, gK, xiK);}, py::arg("gK1"), py::arg("gK"), py::arg("xiK"));

    py::class_<gtsam::DiscreteEulerPoincareHelicopter, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::DiscreteEulerPoincareHelicopter>>(m_, "DiscreteEulerPoincareHelicopter")
        .def(py::init<size_t, size_t, size_t, double, const gtsam::Matrix&, const gtsam::Vector&, double>(), py::arg("xiKey"), py::arg("xiKey_1"), py::arg("gKey"), py::arg("h"), py::arg("Inertia"), py::arg("Fu"), py::arg("m"))
        .def("evaluateError",[](gtsam::DiscreteEulerPoincareHelicopter* self, const gtsam::Vector& xiK, const gtsam::Vector& xiK_1, const gtsam::Pose3& gK){return self->evaluateError(xiK, xiK_1, gK);}, py::arg("xiK"), py::arg("xiK_1"), py::arg("gK"));

    py::class_<gtsam::FixedLagSmootherKeyTimestampMap, boost::shared_ptr<gtsam::FixedLagSmootherKeyTimestampMap>>(m_, "FixedLagSmootherKeyTimestampMap")
        .def(py::init<>())
        .def(py::init<const gtsam::FixedLagSmootherKeyTimestampMap&>(), py::arg("other"))
        .def("size",[](gtsam::FixedLagSmootherKeyTimestampMap* self){return self->size();})
        .def("empty",[](gtsam::FixedLagSmootherKeyTimestampMap* self){return self->empty();})
        .def("clear",[](gtsam::FixedLagSmootherKeyTimestampMap* self){ self->clear();})
        .def("at",[](gtsam::FixedLagSmootherKeyTimestampMap* self, const size_t key){return self->at(key);}, py::arg("key"))
        .def("insert",[](gtsam::FixedLagSmootherKeyTimestampMap* self, const gtsam::FixedLagSmootherKeyTimestampMapValue& value){ self->insert(value);}, py::arg("value"));

    py::class_<gtsam::FixedLagSmootherResult, boost::shared_ptr<gtsam::FixedLagSmootherResult>>(m_, "FixedLagSmootherResult")
        .def("getIterations",[](gtsam::FixedLagSmootherResult* self){return self->getIterations();})
        .def("getNonlinearVariables",[](gtsam::FixedLagSmootherResult* self){return self->getNonlinearVariables();})
        .def("getLinearVariables",[](gtsam::FixedLagSmootherResult* self){return self->getLinearVariables();})
        .def("getError",[](gtsam::FixedLagSmootherResult* self){return self->getError();});

    py::class_<gtsam::FixedLagSmoother, boost::shared_ptr<gtsam::FixedLagSmoother>>(m_, "FixedLagSmoother")
        .def("print",[](gtsam::FixedLagSmoother* self, string s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s"))
        .def("__repr__",
                    [](const gtsam::FixedLagSmoother& self, string s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s"))
        .def("equals",[](gtsam::FixedLagSmoother* self, const gtsam::FixedLagSmoother& rhs, double tol){return self->equals(rhs, tol);}, py::arg("rhs"), py::arg("tol"))
        .def("timestamps",[](gtsam::FixedLagSmoother* self){return self->timestamps();})
        .def("smootherLag",[](gtsam::FixedLagSmoother* self){return self->smootherLag();})
        .def("update",[](gtsam::FixedLagSmoother* self, const gtsam::NonlinearFactorGraph& newFactors, const gtsam::Values& newTheta, const gtsam::FixedLagSmootherKeyTimestampMap& timestamps){return self->update(newFactors, newTheta, timestamps);}, py::arg("newFactors"), py::arg("newTheta"), py::arg("timestamps"))
        .def("update",[](gtsam::FixedLagSmoother* self, const gtsam::NonlinearFactorGraph& newFactors, const gtsam::Values& newTheta, const gtsam::FixedLagSmootherKeyTimestampMap& timestamps, const gtsam::FactorIndices& factorsToRemove){return self->update(newFactors, newTheta, timestamps, factorsToRemove);}, py::arg("newFactors"), py::arg("newTheta"), py::arg("timestamps"), py::arg("factorsToRemove"))
        .def("calculateEstimate",[](gtsam::FixedLagSmoother* self){return self->calculateEstimate();});

    py::class_<gtsam::BatchFixedLagSmoother, gtsam::FixedLagSmoother, boost::shared_ptr<gtsam::BatchFixedLagSmoother>>(m_, "BatchFixedLagSmoother")
        .def(py::init<>())
        .def(py::init<double>(), py::arg("smootherLag"))
        .def(py::init<double, const gtsam::LevenbergMarquardtParams&>(), py::arg("smootherLag"), py::arg("params"))
        .def("print",[](gtsam::BatchFixedLagSmoother* self, string s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s") = "BatchFixedLagSmoother:\n")
        .def("__repr__",
                    [](const gtsam::BatchFixedLagSmoother& self, string s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s") = "BatchFixedLagSmoother:\n")
        .def("params",[](gtsam::BatchFixedLagSmoother* self){return self->params();})
        .def("calculateEstimatePoint2",[](gtsam::BatchFixedLagSmoother* self, size_t key){return self->calculateEstimate<gtsam::Point2>(key);}, py::arg("key"))
        .def("calculateEstimateRot2",[](gtsam::BatchFixedLagSmoother* self, size_t key){return self->calculateEstimate<gtsam::Rot2>(key);}, py::arg("key"))
        .def("calculateEstimatePose2",[](gtsam::BatchFixedLagSmoother* self, size_t key){return self->calculateEstimate<gtsam::Pose2>(key);}, py::arg("key"))
        .def("calculateEstimatePoint3",[](gtsam::BatchFixedLagSmoother* self, size_t key){return self->calculateEstimate<gtsam::Point3>(key);}, py::arg("key"))
        .def("calculateEstimateRot3",[](gtsam::BatchFixedLagSmoother* self, size_t key){return self->calculateEstimate<gtsam::Rot3>(key);}, py::arg("key"))
        .def("calculateEstimatePose3",[](gtsam::BatchFixedLagSmoother* self, size_t key){return self->calculateEstimate<gtsam::Pose3>(key);}, py::arg("key"))
        .def("calculateEstimateCal3_S2",[](gtsam::BatchFixedLagSmoother* self, size_t key){return self->calculateEstimate<gtsam::Cal3_S2>(key);}, py::arg("key"))
        .def("calculateEstimateCal3DS2",[](gtsam::BatchFixedLagSmoother* self, size_t key){return self->calculateEstimate<gtsam::Cal3DS2>(key);}, py::arg("key"))
        .def("calculateEstimateVector",[](gtsam::BatchFixedLagSmoother* self, size_t key){return self->calculateEstimate<gtsam::Vector>(key);}, py::arg("key"))
        .def("calculateEstimateMatrix",[](gtsam::BatchFixedLagSmoother* self, size_t key){return self->calculateEstimate<gtsam::Matrix>(key);}, py::arg("key"));

    py::class_<gtsam::IncrementalFixedLagSmoother, gtsam::FixedLagSmoother, boost::shared_ptr<gtsam::IncrementalFixedLagSmoother>>(m_, "IncrementalFixedLagSmoother")
        .def(py::init<>())
        .def(py::init<double>(), py::arg("smootherLag"))
        .def(py::init<double, const gtsam::ISAM2Params&>(), py::arg("smootherLag"), py::arg("params"))
        .def("print",[](gtsam::IncrementalFixedLagSmoother* self, string s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s") = "IncrementalFixedLagSmoother:\n")
        .def("__repr__",
                    [](const gtsam::IncrementalFixedLagSmoother& self, string s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s") = "IncrementalFixedLagSmoother:\n")
        .def("params",[](gtsam::IncrementalFixedLagSmoother* self){return self->params();})
        .def("getFactors",[](gtsam::IncrementalFixedLagSmoother* self){return self->getFactors();})
        .def("getISAM2",[](gtsam::IncrementalFixedLagSmoother* self){return self->getISAM2();});

    py::class_<gtsam::ConcurrentFilter, boost::shared_ptr<gtsam::ConcurrentFilter>>(m_, "ConcurrentFilter")
        .def("print",[](gtsam::ConcurrentFilter* self, string s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s"))
        .def("__repr__",
                    [](const gtsam::ConcurrentFilter& self, string s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s"))
        .def("equals",[](gtsam::ConcurrentFilter* self, const gtsam::ConcurrentFilter& rhs, double tol){return self->equals(rhs, tol);}, py::arg("rhs"), py::arg("tol"));

    py::class_<gtsam::ConcurrentSmoother, boost::shared_ptr<gtsam::ConcurrentSmoother>>(m_, "ConcurrentSmoother")
        .def("print",[](gtsam::ConcurrentSmoother* self, string s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s"))
        .def("__repr__",
                    [](const gtsam::ConcurrentSmoother& self, string s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s"))
        .def("equals",[](gtsam::ConcurrentSmoother* self, const gtsam::ConcurrentSmoother& rhs, double tol){return self->equals(rhs, tol);}, py::arg("rhs"), py::arg("tol"));

    py::class_<gtsam::ConcurrentBatchFilterResult, boost::shared_ptr<gtsam::ConcurrentBatchFilterResult>>(m_, "ConcurrentBatchFilterResult")
        .def("getIterations",[](gtsam::ConcurrentBatchFilterResult* self){return self->getIterations();})
        .def("getLambdas",[](gtsam::ConcurrentBatchFilterResult* self){return self->getLambdas();})
        .def("getNonlinearVariables",[](gtsam::ConcurrentBatchFilterResult* self){return self->getNonlinearVariables();})
        .def("getLinearVariables",[](gtsam::ConcurrentBatchFilterResult* self){return self->getLinearVariables();})
        .def("getError",[](gtsam::ConcurrentBatchFilterResult* self){return self->getError();});

    py::class_<gtsam::ConcurrentBatchFilter, gtsam::ConcurrentFilter, boost::shared_ptr<gtsam::ConcurrentBatchFilter>>(m_, "ConcurrentBatchFilter")
        .def(py::init<>())
        .def(py::init<const gtsam::LevenbergMarquardtParams&>(), py::arg("parameters"))
        .def("getFactors",[](gtsam::ConcurrentBatchFilter* self){return self->getFactors();})
        .def("getLinearizationPoint",[](gtsam::ConcurrentBatchFilter* self){return self->getLinearizationPoint();})
        .def("getOrdering",[](gtsam::ConcurrentBatchFilter* self){return self->getOrdering();})
        .def("getDelta",[](gtsam::ConcurrentBatchFilter* self){return self->getDelta();})
        .def("update",[](gtsam::ConcurrentBatchFilter* self){return self->update();})
        .def("update",[](gtsam::ConcurrentBatchFilter* self, const gtsam::NonlinearFactorGraph& newFactors){return self->update(newFactors);}, py::arg("newFactors"))
        .def("update",[](gtsam::ConcurrentBatchFilter* self, const gtsam::NonlinearFactorGraph& newFactors, const gtsam::Values& newTheta){return self->update(newFactors, newTheta);}, py::arg("newFactors"), py::arg("newTheta"))
        .def("update",[](gtsam::ConcurrentBatchFilter* self, const gtsam::NonlinearFactorGraph& newFactors, const gtsam::Values& newTheta, const gtsam::KeyList& keysToMove){return self->update(newFactors, newTheta, keysToMove);}, py::arg("newFactors"), py::arg("newTheta"), py::arg("keysToMove"))
        .def("calculateEstimate",[](gtsam::ConcurrentBatchFilter* self){return self->calculateEstimate();});

    py::class_<gtsam::ConcurrentBatchSmootherResult, boost::shared_ptr<gtsam::ConcurrentBatchSmootherResult>>(m_, "ConcurrentBatchSmootherResult")
        .def("getIterations",[](gtsam::ConcurrentBatchSmootherResult* self){return self->getIterations();})
        .def("getLambdas",[](gtsam::ConcurrentBatchSmootherResult* self){return self->getLambdas();})
        .def("getNonlinearVariables",[](gtsam::ConcurrentBatchSmootherResult* self){return self->getNonlinearVariables();})
        .def("getLinearVariables",[](gtsam::ConcurrentBatchSmootherResult* self){return self->getLinearVariables();})
        .def("getError",[](gtsam::ConcurrentBatchSmootherResult* self){return self->getError();});

    py::class_<gtsam::ConcurrentBatchSmoother, gtsam::ConcurrentSmoother, boost::shared_ptr<gtsam::ConcurrentBatchSmoother>>(m_, "ConcurrentBatchSmoother")
        .def(py::init<>())
        .def(py::init<const gtsam::LevenbergMarquardtParams&>(), py::arg("parameters"))
        .def("getFactors",[](gtsam::ConcurrentBatchSmoother* self){return self->getFactors();})
        .def("getLinearizationPoint",[](gtsam::ConcurrentBatchSmoother* self){return self->getLinearizationPoint();})
        .def("getOrdering",[](gtsam::ConcurrentBatchSmoother* self){return self->getOrdering();})
        .def("getDelta",[](gtsam::ConcurrentBatchSmoother* self){return self->getDelta();})
        .def("update",[](gtsam::ConcurrentBatchSmoother* self){return self->update();})
        .def("update",[](gtsam::ConcurrentBatchSmoother* self, const gtsam::NonlinearFactorGraph& newFactors){return self->update(newFactors);}, py::arg("newFactors"))
        .def("update",[](gtsam::ConcurrentBatchSmoother* self, const gtsam::NonlinearFactorGraph& newFactors, const gtsam::Values& newTheta){return self->update(newFactors, newTheta);}, py::arg("newFactors"), py::arg("newTheta"))
        .def("calculateEstimate",[](gtsam::ConcurrentBatchSmoother* self){return self->calculateEstimate();});

    py::class_<gtsam::RelativeElevationFactor, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::RelativeElevationFactor>>(m_, "RelativeElevationFactor")
        .def(py::init<>())
        .def(py::init<size_t, size_t, double, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("poseKey"), py::arg("pointKey"), py::arg("measured"), py::arg("model"))
        .def("measured",[](gtsam::RelativeElevationFactor* self){return self->measured();});

    py::class_<gtsam::DummyFactor, gtsam::NonlinearFactor, boost::shared_ptr<gtsam::DummyFactor>>(m_, "DummyFactor")
        .def(py::init<size_t, size_t, size_t, size_t>(), py::arg("key1"), py::arg("dim1"), py::arg("key2"), py::arg("dim2"));

    py::class_<gtsam::InvDepthFactorVariant1, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::InvDepthFactorVariant1>>(m_, "InvDepthFactorVariant1")
        .def(py::init<size_t, size_t, const gtsam::Point2&, const boost::shared_ptr<gtsam::Cal3_S2>, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("poseKey"), py::arg("landmarkKey"), py::arg("measured"), py::arg("K"), py::arg("model"));

    py::class_<gtsam::InvDepthFactorVariant2, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::InvDepthFactorVariant2>>(m_, "InvDepthFactorVariant2")
        .def(py::init<size_t, size_t, const gtsam::Point2&, const boost::shared_ptr<gtsam::Cal3_S2>, const gtsam::Point3&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("poseKey"), py::arg("landmarkKey"), py::arg("measured"), py::arg("K"), py::arg("referencePoint"), py::arg("model"));

    py::class_<gtsam::InvDepthFactorVariant3a, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::InvDepthFactorVariant3a>>(m_, "InvDepthFactorVariant3a")
        .def(py::init<size_t, size_t, const gtsam::Point2&, const boost::shared_ptr<gtsam::Cal3_S2>, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("poseKey"), py::arg("landmarkKey"), py::arg("measured"), py::arg("K"), py::arg("model"));

    py::class_<gtsam::InvDepthFactorVariant3b, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::InvDepthFactorVariant3b>>(m_, "InvDepthFactorVariant3b")
        .def(py::init<size_t, size_t, size_t, const gtsam::Point2&, const boost::shared_ptr<gtsam::Cal3_S2>, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("poseKey1"), py::arg("poseKey2"), py::arg("landmarkKey"), py::arg("measured"), py::arg("K"), py::arg("model"));

    py::class_<gtsam::Mechanization_bRn2, boost::shared_ptr<gtsam::Mechanization_bRn2>>(m_, "Mechanization_bRn2")
        .def(py::init<>())
        .def(py::init<gtsam::Rot3&, const gtsam::Vector&, const gtsam::Vector&>(), py::arg("initial_bRn"), py::arg("initial_x_g"), py::arg("initial_x_a"))
        .def("b_g",[](gtsam::Mechanization_bRn2* self, double g_e){return self->b_g(g_e);}, py::arg("g_e"))
        .def("bRn",[](gtsam::Mechanization_bRn2* self){return self->bRn();})
        .def("x_g",[](gtsam::Mechanization_bRn2* self){return self->x_g();})
        .def("x_a",[](gtsam::Mechanization_bRn2* self){return self->x_a();})
        .def("correct",[](gtsam::Mechanization_bRn2* self, const gtsam::Vector& dx){return self->correct(dx);}, py::arg("dx"))
        .def("integrate",[](gtsam::Mechanization_bRn2* self, const gtsam::Vector& u, double dt){return self->integrate(u, dt);}, py::arg("u"), py::arg("dt"))
        .def_static("initialize",[](const gtsam::Matrix& U, const gtsam::Matrix& F, double g_e){return gtsam::Mechanization_bRn2::initialize(U, F, g_e);}, py::arg("U"), py::arg("F"), py::arg("g_e"));

    py::class_<gtsam::AHRS, boost::shared_ptr<gtsam::AHRS>>(m_, "AHRS")
        .def(py::init<const gtsam::Matrix&, const gtsam::Matrix&, double>(), py::arg("U"), py::arg("F"), py::arg("g_e"))
        .def("initialize",[](gtsam::AHRS* self, double g_e){return self->initialize(g_e);}, py::arg("g_e"))
        .def("integrate",[](gtsam::AHRS* self, const gtsam::Mechanization_bRn2& mech, boost::shared_ptr<gtsam::GaussianDensity> state, const gtsam::Vector& u, double dt){return self->integrate(mech, state, u, dt);}, py::arg("mech"), py::arg("state"), py::arg("u"), py::arg("dt"))
        .def("aid",[](gtsam::AHRS* self, const gtsam::Mechanization_bRn2& mech, boost::shared_ptr<gtsam::GaussianDensity> state, const gtsam::Vector& f, bool Farrel){return self->aid(mech, state, f, Farrel);}, py::arg("mech"), py::arg("state"), py::arg("f"), py::arg("Farrel"))
        .def("aidGeneral",[](gtsam::AHRS* self, const gtsam::Mechanization_bRn2& mech, boost::shared_ptr<gtsam::GaussianDensity> state, const gtsam::Vector& f, const gtsam::Vector& f_expected, const gtsam::Rot3& increment){return self->aidGeneral(mech, state, f, f_expected, increment);}, py::arg("mech"), py::arg("state"), py::arg("f"), py::arg("f_expected"), py::arg("increment"));

    py::class_<gtsam::DeltaFactor, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::DeltaFactor>>(m_, "DeltaFactor")
        .def(py::init<size_t, size_t, const gtsam::Point2&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("i"), py::arg("j"), py::arg("measured"), py::arg("noiseModel"));

    py::class_<gtsam::DeltaFactorBase, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::DeltaFactorBase>>(m_, "DeltaFactorBase")
        .def(py::init<size_t, size_t, size_t, size_t, const gtsam::Point2&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("b1"), py::arg("i"), py::arg("b2"), py::arg("j"), py::arg("measured"), py::arg("noiseModel"));

    py::class_<gtsam::OdometryFactorBase, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::OdometryFactorBase>>(m_, "OdometryFactorBase")
        .def(py::init<size_t, size_t, size_t, size_t, const gtsam::Pose2&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("b1"), py::arg("i"), py::arg("b2"), py::arg("j"), py::arg("measured"), py::arg("noiseModel"));

    py::class_<gtsam::ProjectionFactorRollingShutter, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::ProjectionFactorRollingShutter>>(m_, "ProjectionFactorRollingShutter")
        .def(py::init<const gtsam::Point2&, double, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t, size_t, const boost::shared_ptr<gtsam::Cal3_S2>>(), py::arg("measured"), py::arg("alpha"), py::arg("noiseModel"), py::arg("poseKey_a"), py::arg("poseKey_b"), py::arg("pointKey"), py::arg("K"))
        .def(py::init<const gtsam::Point2&, double, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t, size_t, const boost::shared_ptr<gtsam::Cal3_S2>, gtsam::Pose3&>(), py::arg("measured"), py::arg("alpha"), py::arg("noiseModel"), py::arg("poseKey_a"), py::arg("poseKey_b"), py::arg("pointKey"), py::arg("K"), py::arg("body_P_sensor"))
        .def(py::init<const gtsam::Point2&, double, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t, size_t, const boost::shared_ptr<gtsam::Cal3_S2>, bool, bool>(), py::arg("measured"), py::arg("alpha"), py::arg("noiseModel"), py::arg("poseKey_a"), py::arg("poseKey_b"), py::arg("pointKey"), py::arg("K"), py::arg("throwCheirality"), py::arg("verboseCheirality"))
        .def(py::init<const gtsam::Point2&, double, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t, size_t, const boost::shared_ptr<gtsam::Cal3_S2>, bool, bool, gtsam::Pose3&>(), py::arg("measured"), py::arg("alpha"), py::arg("noiseModel"), py::arg("poseKey_a"), py::arg("poseKey_b"), py::arg("pointKey"), py::arg("K"), py::arg("throwCheirality"), py::arg("verboseCheirality"), py::arg("body_P_sensor"))
        .def("measured",[](gtsam::ProjectionFactorRollingShutter* self){return self->measured();})
        .def("alpha",[](gtsam::ProjectionFactorRollingShutter* self){return self->alpha();})
        .def("calibration",[](gtsam::ProjectionFactorRollingShutter* self){return self->calibration();})
        .def("verboseCheirality",[](gtsam::ProjectionFactorRollingShutter* self){return self->verboseCheirality();})
        .def("throwCheirality",[](gtsam::ProjectionFactorRollingShutter* self){return self->throwCheirality();})
        .def("serialize", [](gtsam::ProjectionFactorRollingShutter* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::ProjectionFactorRollingShutter* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::ProjectionFactorRollingShutter &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::ProjectionFactorRollingShutter obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::RangeFactor<gtsam::PoseRTV, gtsam::PoseRTV>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::RangeFactor<gtsam::PoseRTV, gtsam::PoseRTV>>>(m_, "RangeFactorRTV")
        .def(py::init<size_t, size_t, double, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key1"), py::arg("key2"), py::arg("measured"), py::arg("noiseModel"))
        .def("serialize", [](gtsam::RangeFactor<gtsam::PoseRTV, gtsam::PoseRTV>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::RangeFactor<gtsam::PoseRTV, gtsam::PoseRTV>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::RangeFactor<gtsam::PoseRTV, gtsam::PoseRTV> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::RangeFactor<gtsam::PoseRTV, gtsam::PoseRTV> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::ProjectionFactorPPP<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::ProjectionFactorPPP<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>>>(m_, "ProjectionFactorPPPCal3_S2")
        .def(py::init<const gtsam::Point2&, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t, size_t, const boost::shared_ptr<gtsam::Cal3_S2>>(), py::arg("measured"), py::arg("noiseModel"), py::arg("poseKey"), py::arg("transformKey"), py::arg("pointKey"), py::arg("k"))
        .def(py::init<const gtsam::Point2&, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t, size_t, const boost::shared_ptr<gtsam::Cal3_S2>, bool, bool>(), py::arg("measured"), py::arg("noiseModel"), py::arg("poseKey"), py::arg("transformKey"), py::arg("pointKey"), py::arg("k"), py::arg("throwCheirality"), py::arg("verboseCheirality"))
        .def("measured",[](gtsam::ProjectionFactorPPP<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>* self){return self->measured();})
        .def("calibration",[](gtsam::ProjectionFactorPPP<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>* self){return self->calibration();})
        .def("verboseCheirality",[](gtsam::ProjectionFactorPPP<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>* self){return self->verboseCheirality();})
        .def("throwCheirality",[](gtsam::ProjectionFactorPPP<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>* self){return self->throwCheirality();})
        .def("serialize", [](gtsam::ProjectionFactorPPP<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::ProjectionFactorPPP<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::ProjectionFactorPPP<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::ProjectionFactorPPP<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::ProjectionFactorPPP<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::ProjectionFactorPPP<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>>>(m_, "ProjectionFactorPPPCal3DS2")
        .def(py::init<const gtsam::Point2&, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t, size_t, const boost::shared_ptr<gtsam::Cal3DS2>>(), py::arg("measured"), py::arg("noiseModel"), py::arg("poseKey"), py::arg("transformKey"), py::arg("pointKey"), py::arg("k"))
        .def(py::init<const gtsam::Point2&, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t, size_t, const boost::shared_ptr<gtsam::Cal3DS2>, bool, bool>(), py::arg("measured"), py::arg("noiseModel"), py::arg("poseKey"), py::arg("transformKey"), py::arg("pointKey"), py::arg("k"), py::arg("throwCheirality"), py::arg("verboseCheirality"))
        .def("measured",[](gtsam::ProjectionFactorPPP<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>* self){return self->measured();})
        .def("calibration",[](gtsam::ProjectionFactorPPP<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>* self){return self->calibration();})
        .def("verboseCheirality",[](gtsam::ProjectionFactorPPP<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>* self){return self->verboseCheirality();})
        .def("throwCheirality",[](gtsam::ProjectionFactorPPP<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>* self){return self->throwCheirality();})
        .def("serialize", [](gtsam::ProjectionFactorPPP<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::ProjectionFactorPPP<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::ProjectionFactorPPP<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::ProjectionFactorPPP<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::ProjectionFactorPPPC<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::ProjectionFactorPPPC<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>>>(m_, "ProjectionFactorPPPCCal3_S2")
        .def(py::init<const gtsam::Point2&, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t, size_t, size_t>(), py::arg("measured"), py::arg("noiseModel"), py::arg("poseKey"), py::arg("transformKey"), py::arg("pointKey"), py::arg("calibKey"))
        .def(py::init<const gtsam::Point2&, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t, size_t, size_t, bool, bool>(), py::arg("measured"), py::arg("noiseModel"), py::arg("poseKey"), py::arg("transformKey"), py::arg("pointKey"), py::arg("calibKey"), py::arg("throwCheirality"), py::arg("verboseCheirality"))
        .def("measured",[](gtsam::ProjectionFactorPPPC<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>* self){return self->measured();})
        .def("verboseCheirality",[](gtsam::ProjectionFactorPPPC<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>* self){return self->verboseCheirality();})
        .def("throwCheirality",[](gtsam::ProjectionFactorPPPC<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>* self){return self->throwCheirality();})
        .def("serialize", [](gtsam::ProjectionFactorPPPC<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::ProjectionFactorPPPC<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::ProjectionFactorPPPC<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::ProjectionFactorPPPC<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::ProjectionFactorPPPC<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::ProjectionFactorPPPC<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>>>(m_, "ProjectionFactorPPPCCal3DS2")
        .def(py::init<const gtsam::Point2&, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t, size_t, size_t>(), py::arg("measured"), py::arg("noiseModel"), py::arg("poseKey"), py::arg("transformKey"), py::arg("pointKey"), py::arg("calibKey"))
        .def(py::init<const gtsam::Point2&, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t, size_t, size_t, bool, bool>(), py::arg("measured"), py::arg("noiseModel"), py::arg("poseKey"), py::arg("transformKey"), py::arg("pointKey"), py::arg("calibKey"), py::arg("throwCheirality"), py::arg("verboseCheirality"))
        .def("measured",[](gtsam::ProjectionFactorPPPC<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>* self){return self->measured();})
        .def("verboseCheirality",[](gtsam::ProjectionFactorPPPC<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>* self){return self->verboseCheirality();})
        .def("throwCheirality",[](gtsam::ProjectionFactorPPPC<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>* self){return self->throwCheirality();})
        .def("serialize", [](gtsam::ProjectionFactorPPPC<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::ProjectionFactorPPPC<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::ProjectionFactorPPPC<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::ProjectionFactorPPPC<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    m_.def("synchronize",[](gtsam::ConcurrentFilter& filter, gtsam::ConcurrentSmoother& smoother){ gtsam::synchronize(filter, smoother);}, py::arg("filter"), py::arg("smoother"));

#include "python/gtsam_unstable/specializations/gtsam_unstable.h"

}

