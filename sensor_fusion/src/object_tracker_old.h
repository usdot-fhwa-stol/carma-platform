#pragma once
/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Torc Robotics, LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Torc Robotics, LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "timer.h"

#include <eigen3/Eigen/Geometry>

#include <boost/math/constants/constants.hpp>
#include <boost/shared_array.hpp>
#include <boost/range/algorithm_ext/push_back.hpp>
#include <boost/range/irange.hpp>

#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <iostream>
#include <cmath>
#include <deque>
#include <unordered_map>
#include <queue>

#include <ros/ros.h>

namespace torc
{

/**
 * @brief This class represents a tracked object.
 */
struct TrackedObject 
{
    enum PresenceVector
    {
        pv_ID = 1 << 0,
        pv_Position = 1 << 1,
        pv_Dimensions = 1 << 2,
        pv_Orientation = 1 << 3,
        pv_LinearVelocity = 1 << 4,
        pv_AngularVelocity = 1 << 5,
        pv_Confidence = 1 << 6,
    };

    TrackedObject() : object_life(0),
                      presence_vector(0),
                      id(0),
                      confidence(0),
                      tracked_score(0),
                      position(Eigen::Vector3d::Zero()),
                      dimensions(Eigen::Vector3d::Zero()),
                      linear_velocity(Eigen::Vector3d::Zero()),
                      orientation(Eigen::Quaterniond::Identity()),
                      angular_velocity(Eigen::Vector3d::Zero())
    {

    }

    std::unordered_map<size_t, boost::shared_array<uint8_t>> src_data;

    float       object_life;
    uint16_t    presence_vector;
    size_t      id;                         // 1
    Eigen::Vector3d     position;           // 2
    Eigen::Vector3d     dimensions;         // 4 // axis-aligned with orientation
    Eigen::Quaterniond  orientation;        // 8
    Eigen::Vector3d     linear_velocity;    // 16
    Eigen::Vector3d     angular_velocity;   // 32
    double      confidence;                 // 64

    double distanceToSquared(const TrackedObject &other) const 
    {
        return (position - other.position).squaredNorm();
    }

    double tracked_score;

private:
    friend class ObjectTracker;
    size_t src_id;
}; // End TrackedObject struct

/**
 * @brief This class tracks objects from multiple sources
 */
class ObjectTracker 
{
    static constexpr double pi = boost::math::constants::pi<double>();

public:
    typedef boost::posix_time::ptime TimeStampType;

    struct 
    {
        /**
         * @brief the decay to apply to object life time. this value is a linear constant to decay the life by
         */
        double life_time_decay;

        /**
         * @brief the score (distance) to consider objects to be paired
         */
        double score_threshold;

        /**
         * @brief The expected max_velocity of an object this is used for associating new dynamic tracks
         */
        double max_velocity;

        /**
         * @brief The score required for a track to be accepted as a track
         */
        double minimum_tracker_score;

        /**
         * @brief The distance required to be associated with a track
         */
        double out_of_range_distance;
    } config;

private:

    /**
     * @brief helper class that is used for internal tracks and measurements
     */
    struct ObjectTrackerSensorObjects 
    {
        size_t src_id;
        TimeStampType time_stamp;
        std::vector<TrackedObject> objects;
    };

    uint16_t last_id_ = 1;
    std::vector<ObjectTrackerSensorObjects> process_q_;
    std::map<size_t, ObjectTrackerSensorObjects> internal_tracks_;
    std::shared_ptr<cav::Timer> timer_;

    /**
     * @brief Updates the tracked object by forwarding its position
     * relative to its velocity and updating the confidence and object_life
     * @param obj
     * @param delta_time
     */
    void update(TrackedObject &obj, double delta_time) 
    {
        obj.position    += obj.linear_velocity    * delta_time;
        obj.object_life -= config.life_time_decay * delta_time;
        obj.confidence  -= config.life_time_decay * delta_time;

        if (obj.confidence < 0) 
        {
            obj.confidence = 0;
        }
    }

    /**
     * @brief Correction step. Corrects using an alpha-beta filter on the measurement
     * @param track
     * @param measurement
     * @param dt
     */
    void correct(TrackedObject &track, const TrackedObject &measurement, double dt) 
    {
        Eigen::Vector3d position_error = measurement.position - track.position;
        Eigen::Vector3d old_position = track.position;

        // Tracked_score can't exceed 1.0, more smoothing = lower score. Max Smoothing: 1.0
        double smoothing = track.tracked_score;

        // Since smoothing < 1.0 alpha inverse proportional to smoothing
        double alpha = 1 - smoothing * smoothing;

        // Beta is proportional to smoothing
        double beta = (1 - smoothing) * (1 - smoothing);

        // If alpha is close to 1, our tracked score is almost nothing so we wait the track less
        track.position += position_error * alpha;

        // Use velocity if the sensor gives it to us
        if (measurement.presence_vector & TrackedObject::pv_LinearVelocity) 
        {
            //ROS_INFO_STREAM("Velocity Diff: " << (track.linear_velocity - measurement.linear_velocity));
            track.linear_velocity = measurement.linear_velocity;
        } 
        else 
        {
            track.linear_velocity += (beta * position_error) / dt;
        }

        if (measurement.presence_vector & TrackedObject::pv_Dimensions) 
        {
            track.dimensions = measurement.dimensions;
        }

        auto old_orientation = track.orientation;
        if (!track.linear_velocity.isZero())
        {
            track.orientation.setFromTwoVectors(track.linear_velocity, Eigen::Vector3d::UnitX());
            if(track.orientation.toRotationMatrix().hasNaN())
            {
                track.orientation.setIdentity();
            }
        }
        else
        {
            track.orientation.setIdentity();
        }

        // Angular Velocity
        if (measurement.presence_vector & TrackedObject::pv_AngularVelocity)
        {
            track.angular_velocity = measurement.angular_velocity;
        }
        else
        {
            // Calculate angular_velocity by change in orientation
            auto r = track.orientation * old_orientation.inverse();
            double theta = 2 * std::acos(r.w());
            if (theta > pi)
            {
                theta -= 2 * pi;
            }

            Eigen::AngleAxisd a(r);
            Eigen::Vector3d v = a.axis();
            track.angular_velocity = (theta / dt) * (v.normalized());
        }

        // Reset object_life to 1.0
        track.object_life = track.object_life + (config.life_time_decay / 10.0);
        if(track.object_life > 1.0)
        {
            track.object_life = 1.0;
        }

        /* Tracked score is an attempt to get a smoother score based on previous confidence the new confidence and the previous
         * tracked_score , tracked_score does not deteriorate directly unless track_confidence gets really low
         * tracked_score also starts at 0 so has to grow before we trust it */
        track.tracked_score = (track.tracked_score + track.confidence + measurement.confidence) / 3.0;
        track.confidence = measurement.confidence;
    }

    /**
     * @brief Process sensor measurments
     *
     * The algorithm is as follows:
     *
     * Update the objects in the sensor_track for dt from previous measurement to new measurement
     *
     * Remove expired tracks
     *
     * Find best candidates for tracks
     * Objects in measurement come from one sensor. Assuming these objects can't pair with more than one track for the
     * sensor
     *
     *
     * @param measurement
     * @param sensor_track
     */

    void processObjects(const ObjectTrackerSensorObjects &measurement, ObjectTrackerSensorObjects &sensor_track) 
    {
        if (measurement.time_stamp < sensor_track.time_stamp)
        {
            return;
        }

        const std::vector<TrackedObject> &measured_objects = measurement.objects;   // Create Vector for measured tracks and fill
        std::vector<TrackedObject> unassociated_tracks;     // Create Vector to put in new tracks
        std::vector<TrackedObject> &tracked_objects = sensor_track.objects;         // Create Vector for sensor tracks and fill


        // Data structures used for processing
        double delta_time = (double)(measurement.time_stamp - sensor_track.time_stamp).total_microseconds() / (double)1000000UL; // Get delta time between measured and sensed
        
        std::for_each(tracked_objects.begin(), tracked_objects.end(), [this, delta_time](TrackedObject &o) { update(o, delta_time); }); // Propagate tracked objects to measurement time_stamp
        purgeExpiredTracks(tracked_objects); // Delete old tracks based on decay and lifetime

        Eigen::MatrixXd score_matrix = Eigen::MatrixXd::Constant(measured_objects.size(), tracked_objects.size(), std::numeric_limits<double>::infinity());  // Create Matrix with Rows: measured_objects size and Columns: tracked_objects size (All values initialized to infinity)

        std::vector<size_t> outofrange_measurements; // Create Vector to store index of measured objects that are outside allowable range
        std::vector<size_t> samerange_measurements; // Create Vector to store index of measured objects that are within range 

        if(tracked_objects.empty())
        {
            ROS_INFO_STREAM("No Existing Tracks, Adding: " << measured_objects.size());
            unassociated_tracks = measured_objects;
        }
        else
        {

            /* Step 1: Create Score Matrix using distances between all Measurement Objects and all Tracked Objects */
            for(size_t measured_idx = 0; measured_idx < measured_objects.size(); measured_idx++)    // Loop through all new measurement objects from sensor
            {
                for(size_t track_idx = 0; track_idx < tracked_objects.size(); track_idx++)   // Loop through all known tracked objects
                {
                    double distSquared = measured_objects[measured_idx].distanceToSquared(tracked_objects[track_idx]); // Calculate Distance from tracked object to measured object
                    score_matrix(measured_idx, track_idx) = distSquared;    // Update specified score matrix cell (measurement, tracked)
                }
            }

            /* Step 2: Go through score matrix and filter out measured objects that are not within range of any tracked objects */
            for(size_t measured_idx = 0; measured_idx < measured_objects.size(); measured_idx++)    // Loop through all new measurement objects from sensor
            {
                bool measured_outofrange = true;    // Create variable to track if any tracked objects are in range of current measured object
                for(size_t track_idx = 0; track_idx < tracked_objects.size(); track_idx++)   // Loop through all known tracked objects
                {
                    if(score_matrix(measured_idx, track_idx) < config.out_of_range_distance)
                    {
                        measured_outofrange = false;    // We have at least one tracked object in range of the current measured object
                        track_idx = tracked_objects.size();     // This will cause us to stop iterating through the tracked objects
                    }
                }
                if(measured_outofrange)
                {
                    outofrange_measurements.push_back(measured_idx);    // Add the measured object to the outofrange_measurements vector
                }
            }

            /* Step 3: Go through score matrix and filter out measured objects that are within a given range of a tracked objects */
            for(size_t track_idx = 0; track_idx < tracked_objects.size(); track_idx++)   // Loop through all known tracked objects
            {
                size_t closest_measured_index  = 0;
                double closest_measured_object = score_matrix(0, track_idx);
                for(size_t measured_idx = 1; measured_idx < measured_objects.size(); measured_idx++)    // Loop through all new measurement objects from sensor
                {
                    if(score_matrix(measured_idx, track_idx) < closest_measured_object)
                    {
                        closest_measured_index  = measured_idx;
                        closest_measured_object = score_matrix(measured_idx, track_idx);
                    }

                    if(score_matrix(measured_idx, track_idx) < config.out_of_range_distance)
                    {
                        samerange_measurements.push_back(measured_idx);     // Add the measured object to the samerange_measurements vector 
                    }
                }
                correct(tracked_objects[track_idx], measured_objects[closest_measured_index], delta_time);  // Correct the Tracked Object based on the closest Measured Object
            }
            
            /* Step 4: Add Measured objects that are contained in the outofrange_measurements vector to the unassociated tracks vector */
            for(size_t outofrange_idx = 0; outofrange_idx < outofrange_measurements.size(); outofrange_idx++)
            {
                bool near_unassociated = false; // Create variable to tell if the current OutOfRange Measured Object is close to an Unassociated Object
                for(size_t unassociated_idx = 0; unassociated_idx < unassociated_tracks.size(); unassociated_idx++)
                {
                    double distSquared = measured_objects[outofrange_measurements[outofrange_idx]].distanceToSquared(unassociated_tracks[unassociated_idx]);
                    if(distSquared < config.out_of_range_distance)
                    {
                        near_unassociated = true;
                        break;   // The current Measured Object is within range of something already in unassociated tracks -> stop checking iterations
                    }
                }
                if(!near_unassociated)
                {
                    unassociated_tracks.push_back(measured_objects[outofrange_measurements[outofrange_idx]]);    // Add this object to the new track list
                    break; // Dont want to add the same measured object twice
                }
            }
        }

        ROS_INFO_STREAM("Tracked: " << tracked_objects.size() << " Measured: " << measured_objects.size() << " New: " << unassociated_tracks.size() << " Ignored (Out-Range): " << (outofrange_measurements.size() - unassociated_tracks.size()) << " Ignored (In-Range): " << samerange_measurements.size());

        /* Step 5: Make Unassociated Objects into New Tracked Objects */
        for (auto &new_object : unassociated_tracks) 
        {
            new_object.object_life = 1.0;
            new_object.tracked_score = 0.0;
            new_object.id = last_id_++;
            tracked_objects.push_back(new_object);
        }

        // Update time stamp
        sensor_track.time_stamp = measurement.time_stamp;
    }

    /**
     * @brief Deletes old tracks
     * @param objs
     */
    void purgeExpiredTracks(std::vector<TrackedObject>& objs)
    {
        size_t before_purge = objs.size();
        objs.erase(std::remove_if(objs.begin(), objs.end(), [](const TrackedObject &obj) { return obj.object_life <= 0.0; }), objs.end());
        size_t after_purge = objs.size();
        ROS_INFO_STREAM("Purging Expired! Start: " << before_purge << " End: " << after_purge);
    }

    /**
     * @brief Gets the nearest neighbours between two disjoint sets
     * @param first
     * @param second
     * @return
     */
    std::unordered_map<size_t ,size_t > getDisjointNeighbours(const std::vector<TrackedObject>& first, const std::vector<TrackedObject>& second)
    {
        std::unordered_map<size_t ,size_t > out;

        for(size_t i = 0; i < first.size(); i++)
        {
            double best_score = std::numeric_limits<double>::infinity();
            size_t best_candidate = std::numeric_limits<size_t>::max();
            for(size_t j = 0; j < second.size(); j++)
            {
                double distSquared = first[i].distanceToSquared(second[j]);
                if(distSquared < best_score)
                {
                    best_candidate = j;
                    best_score = distSquared;
                }
            }

            out[i] = best_candidate;
        }

        return out;
    }

    TrackedObject computeMeanObject(std::vector<TrackedObject>& list)
    {
        TrackedObject obj;
        for(auto& it : list)
        {
            obj.position            += it.tracked_score * it.position;
            obj.linear_velocity     += it.tracked_score * it.linear_velocity;
            obj.angular_velocity    += it.tracked_score * it.angular_velocity;
            obj.tracked_score       += it.tracked_score;
            obj.src_data[it.src_id]  = it.src_data[it.src_id];
        }

        obj.position            /= obj.tracked_score;
        obj.linear_velocity     /= obj.tracked_score;
        obj.angular_velocity    /= obj.tracked_score;
        obj.tracked_score       /= list.size();
        obj.confidence           = obj.tracked_score;

        if (!obj.linear_velocity.isZero())
        {
            obj.orientation.setFromTwoVectors(obj.linear_velocity, Eigen::Vector3d::UnitX());
            if(obj.orientation.toRotationMatrix().hasNaN())
            {
                obj.orientation.setIdentity();
            }
        }
        else
        {
            obj.orientation.setIdentity();
        }

        obj.presence_vector = TrackedObject::pv_ID | TrackedObject::pv_Position | TrackedObject::pv_Orientation | 
                              TrackedObject::pv_LinearVelocity | TrackedObject::pv_AngularVelocity | TrackedObject::pv_Confidence;

        return obj;
    }

    /**
     * @brief Merges the internal tracks
     * @return
     */
    ObjectTrackerSensorObjects mergeTracks()
    {
        ObjectTrackerSensorObjects merged_sensor;
        if(internal_tracks_.empty()) 
        {
            return merged_sensor;
        }

        std::vector<TrackedObject> out_track;

        std::vector<std::vector<TrackedObject>> neighbours_track;

        // We want to get to the latest time_stamp
        TimeStampType latest = internal_tracks_.begin()->second.time_stamp;
        for(auto& it : internal_tracks_)
        {
            if(latest < it.second.time_stamp)
            {
                latest = it.second.time_stamp;
            }
        }

        // Iterate through the internal sensor tracks
        for(auto& it : internal_tracks_)
        {
            // Work on a copy of the objects
            std::vector<TrackedObject> current_objects = it.second.objects;
            double delta_time = (double)(latest - it.second.time_stamp).total_microseconds() / (double)1000000UL;
            std::for_each(current_objects.begin(), current_objects.end(), [this, delta_time](TrackedObject& obj) { update(obj, delta_time); });

            // TODO : should we be expiring tracks here?
            purgeExpiredTracks(current_objects);

            // Remove freshly seen tracks from our processed list (ie tracked_score = 0.0)
            current_objects.erase(std::remove_if(current_objects.begin(), current_objects.end(), [](const TrackedObject &obj) { return obj.tracked_score<= 0.0; }), current_objects.end());

            auto nearest_neighbours = getDisjointNeighbours(current_objects, out_track);
            for(auto& neighbour_pair : nearest_neighbours)
            {
                if(neighbour_pair.second < out_track.size() && current_objects[neighbour_pair.first].distanceToSquared(out_track[neighbour_pair.second]) < config.score_threshold)
                {
                    neighbours_track[neighbour_pair.second].push_back(current_objects[neighbour_pair.first]);
                    out_track[neighbour_pair.second] = computeMeanObject(neighbours_track[neighbour_pair.second]);
                    out_track[neighbour_pair.second].src_id = 0;
                    // ID's are unique no two internal tracks will have the same id. We can use the min of the IDs so that we are consistent between timesteps regardless of ordering
                    out_track[neighbour_pair.second].id = std::min_element(neighbours_track[neighbour_pair.second].begin(), neighbours_track[neighbour_pair.second].end(),
                                                                           [](const torc::TrackedObject& a, const torc::TrackedObject& b) {return a.id < b.id;} )->id;
                }
                else
                {
                    neighbours_track.emplace_back();
                    neighbours_track.back().push_back(current_objects[neighbour_pair.first]);
                    out_track.push_back(current_objects[neighbour_pair.first]);
                }
            }
        }

        merged_sensor.src_id = 0;
        merged_sensor.time_stamp = latest;
        merged_sensor.objects = std::move(out_track);

        return merged_sensor;
    }

public:

    std::unique_ptr<ObjectTrackerSensorObjects> tracked_sensor_;
    explicit ObjectTracker(std::shared_ptr<cav::Timer> timer = std::make_shared<cav::Timer>()) : timer_(timer), config(), tracked_sensor_(new ObjectTrackerSensorObjects()) 
    {
        tracked_sensor_->time_stamp = timer_->getTime();
    }

    virtual ~ObjectTracker() {}

    void reset()
    {
        tracked_sensor_.reset(new ObjectTrackerSensorObjects());
        tracked_sensor_->time_stamp = timer_->getTime();
    }

    template<class InputIterator>
    void addObjects(InputIterator begin, InputIterator end, size_t src_id, TimeStampType time_stamp)
    {
        ObjectTrackerSensorObjects objs;
        objs.src_id = src_id;
        objs.time_stamp = time_stamp;
        objs.objects.insert(objs.objects.begin(), begin, end);
        std::for_each(objs.objects.begin(), objs.objects.end(), [src_id](TrackedObject&obj){ obj.src_id = src_id; });
        process_q_.emplace_back(std::move(objs));
    }

    size_t process() 
    {
        if (process_q_.empty()) 
        {
            return 0;
        }

        // Sort measurements from lowest time_stamp
        std::sort(process_q_.begin(), process_q_.end(), [](const ObjectTrackerSensorObjects &left, const ObjectTrackerSensorObjects &right) { return left.time_stamp < right.time_stamp; });

        // Process measurements
        ROS_INFO_STREAM("Sensor Process Objects");
        std::for_each(process_q_.begin(), process_q_.end(), [this](const ObjectTrackerSensorObjects &obj) { auto internal_track = internal_tracks_.find(obj.src_id);
                          // Have we started tracking this sensor yet
                          if (internal_track == internal_tracks_.end()) 
                          {
                              ObjectTrackerSensorObjects new_internal_track;
                              new_internal_track.src_id = obj.src_id;
                              new_internal_track.time_stamp = obj.time_stamp;
                              auto ret = internal_tracks_.insert(std::make_pair(obj.src_id, new_internal_track));
                              internal_track = ret.first;
                          }
                          processObjects(obj, internal_track->second);
                      });

        size_t size = process_q_.size();
        process_q_.clear();

        ObjectTrackerSensorObjects merged_measurement = mergeTracks();

        // We want the tracked_score to be 0 so we get ID association but dont do any "real" tracking
        if(tracked_sensor_->objects.empty())
        {
            tracked_sensor_->src_id = merged_measurement.src_id;
            tracked_sensor_->time_stamp = merged_measurement.time_stamp;
        }
        // else
        // {
            // std::for_each(tracked_sensor_->objects.begin(), tracked_sensor_->objects.end(), [](TrackedObject& obj){ obj.tracked_score = 0.0; });
        // }
        ROS_INFO_STREAM("Merge Process Objects");
        processObjects(merged_measurement, *tracked_sensor_);

        return size;
    }


//gen.add("out_of_range_distance", double_t, 0, "The distance required to be associated with a track", 0.5 , 0, 10.0)
    
}; // End ObjectTracker class
} // End torc namespace
