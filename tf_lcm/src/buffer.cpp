#include "tf_lcm/buffer.hpp"
#include <algorithm>
#include <queue>
#include <iostream>
#include <thread>
#include <sstream>
#include <cmath>

// Macro to print debug messages only when debugging is enabled
#define TF_DEBUG(buffer, msg) \
    if ((buffer).debug_enabled_) { std::cerr << "DEBUG: " << msg << std::endl; }

namespace tf_lcm {

Buffer::Buffer(double cache_time)
    : cache_time_(cache_time),
      cached_most_recent_timestamp_(std::chrono::system_clock::now()),
      cached_timestamp_valid_(false),
      debug_enabled_(false) {
}

Buffer::~Buffer() {
    clear();
}

bool Buffer::setTransform(const geometry_msgs::TransformStamped& transform, const std::string& authority, bool is_static) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Check for valid transform
    if (transform.child_frame_id == transform.header.frame_id) {
        std::cerr << "TF_SELF_TRANSFORM: Ignoring transform with frame_id and child_frame_id: " 
                  << transform.child_frame_id << std::endl;
        return false;
    }
    
    if (transform.child_frame_id.empty()) {
        std::cerr << "TF_NO_CHILD_FRAME_ID: Ignoring transform with empty child_frame_id" << std::endl;
        return false;
    }
    
    if (transform.header.frame_id.empty()) {
        std::cerr << "TF_NO_FRAME_ID: Ignoring transform with empty frame_id" << std::endl;
        return false;
    }
    
    // Store the transform
    TransformStorage storage(transform, authority);
    
    // Update cached most recent timestamp if this transform is newer
    auto transform_time = storage.stamp;
    if (!cached_timestamp_valid_ || transform_time > cached_most_recent_timestamp_) {
        cached_most_recent_timestamp_ = transform_time;
        cached_timestamp_valid_ = true;
    }
    
    auto& buffer = is_static ? static_buffer_ : buffer_;
    
    // Store in buffer, parent->child
    buffer[transform.header.frame_id][transform.child_frame_id].push_back(storage);
    
    // Clean old transforms
    auto now = std::chrono::system_clock::now();
    for (auto& parent_map : buffer) {
        for (auto& child_vec : parent_map.second) {
            auto& transforms = child_vec.second;
            
            // Keep at least one transform regardless of age - crucial for log playback
            if (transforms.size() > 1) {
                // Sort by timestamp, newest first
                std::sort(transforms.begin(), transforms.end(), 
                    [](const TransformStorage& a, const TransformStorage& b) {
                        return a.stamp > b.stamp;
                    });
                
                // Remove old transforms but keep at least one
                auto erase_begin = transforms.begin() + 1; // Keep the newest one
                auto erase_end = std::remove_if(erase_begin, transforms.end(),
                    [this, now, is_static](const TransformStorage& ts) {
                        return !is_static && 
                               std::chrono::duration<double>(now - ts.stamp).count() > cache_time_;
                    });
                transforms.erase(erase_begin, erase_end);
            }
        }
    }
    
    return true;
}

bool Buffer::setTransforms(const std::vector<geometry_msgs::TransformStamped>& transforms, const std::string& authority, bool is_static) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    bool result = true;
    for (const auto& transform : transforms) {
        result = result && setTransform(transform, authority, is_static);
    }
    
    return result;
}

geometry_msgs::TransformStamped Buffer::lookupTransform(
    const std::string& target_frame,
    const std::string& source_frame,
    const std::chrono::system_clock::time_point& time,
    const std::chrono::duration<double>& timeout,
    const std::chrono::duration<double>& time_tolerance) {
    
    // Check for timeout
    if (timeout > std::chrono::duration<double>(0.0)) {
        _waitForTransform(target_frame, source_frame, time, timeout);
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    return _lookupTransformNoLock(target_frame, source_frame, time, time_tolerance);
}

geometry_msgs::TransformStamped Buffer::lookupTransform(
    const std::string& target_frame,
    const std::chrono::system_clock::time_point& target_time,
    const std::string& source_frame,
    const std::chrono::system_clock::time_point& source_time,
    const std::string& fixed_frame,
    const std::chrono::duration<double>& timeout) {
    
    // TODO: Implement advanced time travel
    // This is a simplified version
    
    // Look up transform from fixed frame to source at source_time
    auto source_to_fixed = lookupTransform(fixed_frame, source_frame, source_time, timeout);
    
    // Look up transform from fixed frame to target at target_time
    auto target_to_fixed = lookupTransform(fixed_frame, target_frame, target_time, timeout);
    
    // Invert the target to fixed transform to get fixed to target
    // Normally we would implement proper transform inversion, but for simplicity we'll just assume it's possible
    
    // Combine the transforms: source -> fixed -> target
    // Again, normally we'd implement proper transform composition
    
    // For now, just return a placeholder transform
    geometry_msgs::TransformStamped result;
    result.header.frame_id = target_frame;
    result.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(target_time.time_since_epoch()).count();
    result.header.stamp.nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(
        target_time.time_since_epoch() % std::chrono::seconds(1)).count();
    result.child_frame_id = source_frame;
    result.transform = target_to_fixed.transform; // This is just a placeholder
    
    return result;
}

bool Buffer::canTransform(
    const std::string& target_frame,
    const std::string& source_frame,
    const std::chrono::system_clock::time_point& time,
    std::string* error_msg,
    const std::chrono::duration<double>& time_tolerance) {
    
    std::lock_guard<std::mutex> lock(mutex_);
    return _canTransformNoLock(target_frame, source_frame, time, error_msg, time_tolerance);
}

bool Buffer::canTransform(
    const std::string& target_frame,
    const std::chrono::system_clock::time_point& target_time,
    const std::string& source_frame,
    const std::chrono::system_clock::time_point& source_time,
    const std::string& fixed_frame,
    std::string* error_msg) {
    
    // Check if we can transform from source to fixed
    if (!canTransform(fixed_frame, source_frame, source_time, error_msg)) {
        return false;
    }
    
    // Check if we can transform from fixed to target
    if (!canTransform(fixed_frame, target_frame, target_time, error_msg)) {
        return false;
    }
    
    return true;
}

std::vector<std::string> Buffer::getAllFrameNames() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<std::string> frames;
    
    // Add static frames
    for (const auto& parent_map : static_buffer_) {
        frames.push_back(parent_map.first);
        for (const auto& child_vec : parent_map.second) {
            frames.push_back(child_vec.first);
        }
    }
    
    // Add non-static frames
    for (const auto& parent_map : buffer_) {
        frames.push_back(parent_map.first);
        for (const auto& child_vec : parent_map.second) {
            frames.push_back(child_vec.first);
        }
    }
    
    // Remove duplicates
    std::sort(frames.begin(), frames.end());
    frames.erase(std::unique(frames.begin(), frames.end()), frames.end());
    
    return frames;
}

void Buffer::setDebugEnabled(bool enabled) {
    debug_enabled_ = enabled;
}

void Buffer::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_.clear();
    static_buffer_.clear();
    
    // Invalidate the cached timestamp since the buffer is now empty
    cached_timestamp_valid_ = false;
}

bool Buffer::_canTransformNoLock(
    const std::string& target_frame,
    const std::string& source_frame,
    const std::chrono::system_clock::time_point& time,
    std::string* error_msg,
    const std::chrono::duration<double>& time_tolerance) {
    
    // AUTO-DETECT LOG PLAYBACK MODE
    // Check if we have any transforms with significantly older timestamps (log playback)
    bool log_playback_mode = false;
    auto now = std::chrono::system_clock::now();
    double time_since_now = std::abs(std::chrono::duration<double>(time - now).count());
    if (time_since_now > 1000.0) {
        // If requested time is more than 1000 seconds different from current time,
        // we're almost certainly in log playback mode
        log_playback_mode = true;
        TF_DEBUG(*this, "Detected log playback mode (requested time differs from now by " 
                 << time_since_now << " seconds)");
    }
    
    // If frames are the same, we can transform
    if (target_frame == source_frame) {
        return true;
    }
    
    // Check for direct transform in forward direction
    if (buffer_.count(target_frame) && buffer_[target_frame].count(source_frame)) {
        auto& transforms = buffer_[target_frame][source_frame];
        if (!transforms.empty()) {
            // For log playback mode, just return true if any transform exists
            if (log_playback_mode || time_tolerance.count() > 1000.0) {
                TF_DEBUG(*this, "Found forward transform in log playback mode, using it");
                return true;
            }
            
            // Otherwise check if any transform is within the time tolerance
            for (const auto& transform : transforms) {
                double time_diff = std::abs(std::chrono::duration<double>(transform.stamp - time).count());
                if (time_diff <= time_tolerance.count()) {
                    return true;
                }
            }
        }
    }
    
    // Check for direct transform in reverse direction
    if (buffer_.count(source_frame) && buffer_[source_frame].count(target_frame)) {
        auto& transforms = buffer_[source_frame][target_frame];
        if (!transforms.empty()) {
            // For log playback with large time tolerance, just return true if any transform exists
            if (time_tolerance.count() > 1000.0) {
                return true;
            }
            
            // Otherwise check if any transform is within the time tolerance
            for (const auto& transform : transforms) {
                double time_diff = std::abs(std::chrono::duration<double>(transform.stamp - time).count());
                if (time_diff <= time_tolerance.count()) {
                    return true;
                }
            }
        }
    }
    
    // Check static transforms in forward direction
    // Static transforms don't need time tolerance checks since they're always valid,
    // but we'll use the same pattern for consistency
    if (static_buffer_.count(target_frame) && static_buffer_[target_frame].count(source_frame)) {
        auto& transforms = static_buffer_[target_frame][source_frame];
        if (!transforms.empty()) {
            return true;
        }
    }
    
    // Check static transforms in reverse direction
    if (static_buffer_.count(source_frame) && static_buffer_[source_frame].count(target_frame)) {
        auto& transforms = static_buffer_[source_frame][target_frame];
        if (!transforms.empty()) {
            return true;
        }
    }
    
    // Implement graph search to find multi-hop paths
    // Collect all frames
    std::vector<std::string> all_frames;
    std::unordered_map<std::string, std::vector<std::string>> graph;
    
    // Build graph from dynamic buffer
    for (const auto& parent_map : buffer_) {
        if (std::find(all_frames.begin(), all_frames.end(), parent_map.first) == all_frames.end()) {
            all_frames.push_back(parent_map.first);
        }
        
        for (const auto& child_map : parent_map.second) {
            if (std::find(all_frames.begin(), all_frames.end(), child_map.first) == all_frames.end()) {
                all_frames.push_back(child_map.first);
            }
            
            // Add edges in both directions for full connectivity
            graph[parent_map.first].push_back(child_map.first);
            graph[child_map.first].push_back(parent_map.first);
        }
    }
    
    // Build graph from static buffer
    for (const auto& parent_map : static_buffer_) {
        if (std::find(all_frames.begin(), all_frames.end(), parent_map.first) == all_frames.end()) {
            all_frames.push_back(parent_map.first);
        }
        
        for (const auto& child_map : parent_map.second) {
            if (std::find(all_frames.begin(), all_frames.end(), child_map.first) == all_frames.end()) {
                all_frames.push_back(child_map.first);
            }
            
            // Add edges in both directions for full connectivity
            graph[parent_map.first].push_back(child_map.first);
            graph[child_map.first].push_back(parent_map.first);
        }
    }
    
    // Perform BFS to find a path
    std::queue<std::string> q;
    std::unordered_map<std::string, bool> visited;
    
    q.push(source_frame);
    visited[source_frame] = true;
    
    while (!q.empty()) {
        std::string current = q.front();
        q.pop();
        
        // Check if we've reached the target
        if (current == target_frame) {
            return true;
        }
        
        // Visit all neighbors
        for (const auto& neighbor : graph[current]) {
            if (!visited[neighbor]) {
                visited[neighbor] = true;
                q.push(neighbor);
            }
        }
    }
    
    if (error_msg) {
        std::stringstream ss;
        ss << "Cannot transform from frame '" << source_frame << "' to '" << target_frame << "'";
        *error_msg = ss.str();
    }
    
    return false;
}

geometry_msgs::TransformStamped Buffer::_lookupTransformNoLock(
    const std::string& target_frame,
    const std::string& source_frame,
    const std::chrono::system_clock::time_point& time,
    const std::chrono::duration<double>& time_tolerance) {
    
    // If frames are the same, return identity transform
    if (target_frame == source_frame) {
        geometry_msgs::TransformStamped identity;
        identity.header.frame_id = target_frame;
        identity.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(time.time_since_epoch()).count();
        identity.header.stamp.nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(
            time.time_since_epoch() % std::chrono::seconds(1)).count();
        identity.child_frame_id = source_frame;
        identity.transform.rotation.w = 1.0;
        return identity;
    }
    
    // Check if we have direct transform in either buffer
    if ((buffer_.count(source_frame) && buffer_[source_frame].count(target_frame)) ||
        (static_buffer_.count(source_frame) && static_buffer_[source_frame].count(target_frame))) {
        // We have a direct transform from source to target, invert it
        auto direct_transform = _lookupDirectTransform(source_frame, target_frame, time);
        return _inverseTransform(direct_transform);
    }
    
    // Check for direct transform in dynamic buffer
    if (buffer_.count(target_frame) && buffer_[target_frame].count(source_frame)) {
        // Find closest transform in time
        auto& transforms = buffer_[target_frame][source_frame];
        if (!transforms.empty()) {
            // Find transform closest to requested time
            auto closest = std::min_element(transforms.begin(), transforms.end(),
                [time](const TransformStorage& a, const TransformStorage& b) {
                    return std::abs(std::chrono::duration<double>(a.stamp - time).count()) <
                           std::abs(std::chrono::duration<double>(b.stamp - time).count());
                });
            
            return closest->toTransformStamped();
        }
    }
    
    // Check static transforms
    if (static_buffer_.count(target_frame) && static_buffer_[target_frame].count(source_frame)) {
        auto& transforms = static_buffer_[target_frame][source_frame];
        if (!transforms.empty()) {
            return transforms[0].toTransformStamped();
        }
    }
    
    // If we get here, we need to perform a graph search to find a path through multiple transforms
    // Implementation uses breadth-first search to find the shortest path through the transform tree
    
    // Collect all frames
    std::vector<std::string> all_frames;
    std::unordered_map<std::string, std::vector<std::string>> graph;
    
    // Build graph from dynamic buffer
    for (const auto& parent_map : buffer_) {
        if (std::find(all_frames.begin(), all_frames.end(), parent_map.first) == all_frames.end()) {
            all_frames.push_back(parent_map.first);
        }
        
        for (const auto& child_map : parent_map.second) {
            if (std::find(all_frames.begin(), all_frames.end(), child_map.first) == all_frames.end()) {
                all_frames.push_back(child_map.first);
            }
            
            // Add edges in both directions for full connectivity
            graph[parent_map.first].push_back(child_map.first);
            graph[child_map.first].push_back(parent_map.first);
        }
    }
    
    // Build graph from static buffer
    for (const auto& parent_map : static_buffer_) {
        if (std::find(all_frames.begin(), all_frames.end(), parent_map.first) == all_frames.end()) {
            all_frames.push_back(parent_map.first);
        }
        
        for (const auto& child_map : parent_map.second) {
            if (std::find(all_frames.begin(), all_frames.end(), child_map.first) == all_frames.end()) {
                all_frames.push_back(child_map.first);
            }
            
            // Add edges in both directions for full connectivity
            graph[parent_map.first].push_back(child_map.first);
            graph[child_map.first].push_back(parent_map.first);
        }
    }
    
    // Perform BFS to find shortest path
    std::queue<std::string> q;
    std::unordered_map<std::string, std::string> parent;
    std::unordered_map<std::string, bool> visited;
    
    q.push(source_frame);
    visited[source_frame] = true;
    
    bool found_path = false;
    
    while (!q.empty() && !found_path) {
        std::string current = q.front();
        q.pop();
        
        for (const auto& neighbor : graph[current]) {
            if (!visited[neighbor]) {
                visited[neighbor] = true;
                parent[neighbor] = current;
                q.push(neighbor);
                
                if (neighbor == target_frame) {
                    found_path = true;
                    break;
                }
            }
        }
    }
    
    if (!found_path) {
        throw LookupException("Cannot find transform from '" + source_frame + "' to '" + target_frame + "'");
    }
    
    // Reconstruct path from source to target
    std::vector<std::pair<std::string, std::string>> path;
    std::string current = target_frame;
    
    while (current != source_frame) {
        path.push_back({parent[current], current});
        current = parent[current];
    }
    
    // Reverse path to get from source to target
    std::reverse(path.begin(), path.end());
    
    // Apply transforms along the path
    geometry_msgs::TransformStamped result;
    bool first = true;
    
    // Debug information
    std::stringstream path_ss;
    path_ss << "Transform path: ";
    for (const auto& edge : path) {
        path_ss << edge.first << " -> " << edge.second << ", ";
    }
    
    for (const auto& edge : path) {
        geometry_msgs::TransformStamped transform;
        
        // The frame order matters: we want to go from parent to child in our path
        // Check if direct transform exists first
        if ((buffer_.count(edge.first) && buffer_[edge.first].count(edge.second)) ||
            (static_buffer_.count(edge.first) && static_buffer_[edge.first].count(edge.second))) {
            // Direct transform from parent to child
            transform = _lookupDirectTransform(edge.first, edge.second, time);
        } else {
            // Try reverse direction and invert
            transform = _inverseTransform(_lookupDirectTransform(edge.second, edge.first, time));
        }
        
        if (first) {
            result = transform;
            first = false;
        } else {
            // Compose transforms
            result = _composeTransforms(result, transform);
        }
    }
    
    // Set final result headers correctly
    // The transform returned should represent transform from target_frame to source_frame
    // That is, it transforms a point in the target_frame coordinate system to the source_frame system
    // But the path we constructed is from source to target, so we need to invert the final result
    auto final_transform = _inverseTransform(result);
    
    // Set the correct frames
    final_transform.header.frame_id = target_frame;
    final_transform.child_frame_id = source_frame;
    final_transform.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(time.time_since_epoch()).count();
    final_transform.header.stamp.nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(
        time.time_since_epoch() % std::chrono::seconds(1)).count();
    
    return final_transform;
}

void Buffer::_waitForTransform(
    const std::string& target_frame,
    const std::string& source_frame,
    const std::chrono::system_clock::time_point& time,
    const std::chrono::duration<double>& timeout) {
    
    auto start_time = std::chrono::system_clock::now();
    
    // Check periodically until timeout
    while (std::chrono::system_clock::now() - start_time < timeout) {
        std::string error_msg;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            // Use the same large time tolerance for log playback (10000 seconds)
            const std::chrono::duration<double> large_time_tolerance(10000.0);
            if (_canTransformNoLock(target_frame, source_frame, time, &error_msg, large_time_tolerance)) {
                return;
            }
        }
        
        // Sleep to avoid busy waiting
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    throw TimeoutException("Timeout waiting for transform from '" + 
                          source_frame + "' to '" + target_frame + "'");
}

geometry_msgs::TransformStamped Buffer::_lookupDirectTransform(
    const std::string& source_frame,
    const std::string& target_frame,
    const std::chrono::system_clock::time_point& time,
    const std::chrono::duration<double>& time_tolerance) {
    
    // Debug log for this critical lookup
    TF_DEBUG(*this, "Looking for direct transform: '" << source_frame << "' -> '" << target_frame << "'");
    
    // Check dynamic buffer
    if (buffer_.count(source_frame) && buffer_[source_frame].count(target_frame)) {
        TF_DEBUG(*this, "Found direct mapping in buffer: " << source_frame << " -> " << target_frame);
        auto& transforms = buffer_[source_frame][target_frame];
        if (!transforms.empty()) {
            // Find transform closest to requested time
            auto closest = std::min_element(transforms.begin(), transforms.end(),
                [time](const TransformStorage& a, const TransformStorage& b) {
                    return std::abs(std::chrono::duration<double>(a.stamp - time).count()) <
                           std::abs(std::chrono::duration<double>(b.stamp - time).count());
                });
            
            // Check if the closest transform is within our time tolerance
            double time_diff = std::abs(std::chrono::duration<double>(closest->stamp - time).count());
            TF_DEBUG(*this, "Found transform with time diff: " << time_diff << ", tolerance: " << time_tolerance.count());
            
            // With large tolerance for log playback, just use the transform regardless of time
            // This applies to any difference larger than ~2 hours (typical for log playback scenarios)
            if (time_diff > 1000.0 || time_tolerance.count() > 1000.0) {
                TF_DEBUG(*this, "Using transform despite large time difference (log playback mode)");
                return closest->toTransformStamped();
            }
            
            // If within tolerance, use it
            if (time_diff <= time_tolerance.count()) {
                TF_DEBUG(*this, "Using transform within tolerance");
                return closest->toTransformStamped();
            }
            
            // If the time difference is too large, check if ANY transform in the buffer is within tolerance
            for (const auto& transform : transforms) {
                time_diff = std::abs(std::chrono::duration<double>(transform.stamp - time).count());
                if (time_diff <= time_tolerance.count()) {
                    TF_DEBUG(*this, "Found another transform within tolerance");
                    return transform.toTransformStamped();
                }
            }
        }
    }
    
    // SECOND ATTEMPT: Look for target -> source in dynamic buffer and invert it
    if (buffer_.count(target_frame) && buffer_[target_frame].count(source_frame)) {
        TF_DEBUG(*this, "Found REVERSE mapping in buffer: " << target_frame << " -> " << source_frame);
        auto& transforms = buffer_[target_frame][source_frame];
        if (!transforms.empty()) {
            // Find transform closest to requested time
            auto closest = std::min_element(transforms.begin(), transforms.end(),
                [time](const TransformStorage& a, const TransformStorage& b) {
                    return std::abs(std::chrono::duration<double>(a.stamp - time).count()) <
                           std::abs(std::chrono::duration<double>(b.stamp - time).count());
                });
            
            // With large tolerance for log playback, just use the transform regardless of time
            // This applies to any difference larger than ~2 hours (typical for log playback scenarios)
            double inverse_time_diff = std::abs(std::chrono::duration<double>(closest->stamp - time).count());
            TF_DEBUG(*this, "Found INVERSE transform with time diff: " << inverse_time_diff << ", tolerance: " << time_tolerance.count());
            
            if (inverse_time_diff > 1000.0 || time_tolerance.count() > 1000.0) {
                TF_DEBUG(*this, "Using INVERSE transform despite large time difference (log playback mode)");
                // Invert the transform since we found target -> source but need source -> target
                auto transform = closest->toTransformStamped();
                return _inverseTransform(transform);
            }
            
            // Check if the closest transform is within our time tolerance
            double time_diff = std::abs(std::chrono::duration<double>(closest->stamp - time).count());
            
            // If within tolerance, use it
            if (time_diff <= time_tolerance.count()) {
                TF_DEBUG(*this, "Using INVERSE transform within tolerance");
                // Invert the transform since we found target -> source but need source -> target
                auto transform = closest->toTransformStamped();
                return _inverseTransform(transform);
            }
            
            // If the time difference is too large, check if ANY transform in the buffer is within tolerance
            for (const auto& transform_storage : transforms) {
                time_diff = std::abs(std::chrono::duration<double>(transform_storage.stamp - time).count());
                if (time_diff <= time_tolerance.count()) {
                    TF_DEBUG(*this, "Found another INVERSE transform within tolerance");
                    // Invert the transform
                    auto transform = transform_storage.toTransformStamped();
                    return _inverseTransform(transform);
                }
            }
        }
    }
    
    // THIRD ATTEMPT: Check static buffer for source -> target
    if (static_buffer_.count(source_frame) && static_buffer_[source_frame].count(target_frame)) {
        TF_DEBUG(*this, "Found static transform: " << source_frame << " -> " << target_frame);
        auto& transforms = static_buffer_[source_frame][target_frame];
        if (!transforms.empty()) {
            return transforms[0].toTransformStamped();
        }
    }
    
    // FOURTH ATTEMPT: Check static buffer for target -> source and invert
    if (static_buffer_.count(target_frame) && static_buffer_[target_frame].count(source_frame)) {
        TF_DEBUG(*this, "Found static INVERSE transform: " << target_frame << " -> " << source_frame);
        auto& transforms = static_buffer_[target_frame][source_frame];
        if (!transforms.empty()) {
            // Invert the transform
            auto transform = transforms[0].toTransformStamped();
            return _inverseTransform(transform);
        }
    }
    
    // FINAL ATTEMPT: Dump all available transforms for debugging
    TF_DEBUG(*this, "No transform found. Dumping available transforms:");
    if (debug_enabled_) {
        for (const auto& parent_pair : buffer_) {
            for (const auto& child_pair : parent_pair.second) {
                std::cerr << "  Buffer contains: " << parent_pair.first << " -> " << child_pair.first << " (" << child_pair.second.size() << " transforms)" << std::endl;
            }
        }
        
        for (const auto& parent_pair : static_buffer_) {
            for (const auto& child_pair : parent_pair.second) {
                std::cerr << "  Static buffer contains: " << parent_pair.first << " -> " << child_pair.first << " (" << child_pair.second.size() << " transforms)" << std::endl;
            }
        }
    }
    
    throw LookupException("Cannot find direct transform from '" + source_frame + "' to '" + target_frame + "'");
}
geometry_msgs::TransformStamped Buffer::_inverseTransform(
    const geometry_msgs::TransformStamped& transform) {
    
    geometry_msgs::TransformStamped inverted;
    
    // Swap frame IDs
    inverted.header.frame_id = transform.child_frame_id;
    inverted.child_frame_id = transform.header.frame_id;
    
    // Copy timestamp
    inverted.header.stamp = transform.header.stamp;
    
    // Extract original transform values
    double tx = transform.transform.translation.x;
    double ty = transform.transform.translation.y;
    double tz = transform.transform.translation.z;
    
    double qw = transform.transform.rotation.w;
    double qx = transform.transform.rotation.x;
    double qy = transform.transform.rotation.y;
    double qz = transform.transform.rotation.z;
    
    // Debug output original
    std::cout << "Original transform:" << std::endl;
    std::cout << "  Frame: " << transform.header.frame_id << " -> " << transform.child_frame_id << std::endl;
    std::cout << "  Rotation (w,x,y,z): " << qw << " " << qx << " " << qy << " " << qz << std::endl;
    std::cout << "  Translation (x,y,z): " << tx << " " << ty << " " << tz << std::endl;
    
    // Create rotation matrix from quaternion - similar to tf2::Matrix3x3 setRotation
    // This will help us visualize what's happening with rotation
    double s = qw*qw + qx*qx + qy*qy + qz*qz;    // squared length
    
    if (s < 1e-6) {
        // Default to identity for near-zero length
        s = 1.0;
        qw = 1.0;
        qx = qy = qz = 0.0;
    } else {
        // Normalize quaternion if needed
        s = 2.0 / s;  // This is the normalizing factor times 2 used in Matrix3x3::setRotation
    }
    
    // Create rotation matrix for debug visualization
    double xs = qx * s,   ys = qy * s,   zs = qz * s;
    double wx = qw * xs,  wy = qw * ys,  wz = qw * zs;
    double xx = qx * xs,  xy = qx * ys,  xz = qx * zs;
    double yy = qy * ys,  yz = qy * zs,  zz = qz * zs;
    
    std::cout << "  Rotation Matrix:" << std::endl;
    std::cout << "    [" << (1.0 - (yy + zz)) << ", " << (xy - wz) << ", " << (xz + wy) << "]" << std::endl;
    std::cout << "    [" << (xy + wz) << ", " << (1.0 - (xx + zz)) << ", " << (yz - wx) << "]" << std::endl;
    std::cout << "    [" << (xz - wy) << ", " << (yz + wx) << ", " << (1.0 - (xx + yy)) << "]" << std::endl;
    
    // 1. Invert rotation - for unit quaternion, the inverse is the conjugate
    double inv_qw = qw;
    double inv_qx = -qx;
    double inv_qy = -qy;
    double inv_qz = -qz;
    
    // 2. Negate and rotate translation
    double nx = -tx;
    double ny = -ty;
    double nz = -tz;
    
    // Apply rotation using q * v * q^-1 (quatRotate from tf2)
    // Using the expanded formula for rotating a vector by a quaternion
    double tw2 = qw*qw;
    double tx2 = qx*qx;
    double ty2 = qy*qy;
    double tz2 = qz*qz;
    
    // Calculate components of quaternion rotation matrix
    double t0 = tx2 + ty2 + tz2 + tw2;  // Should be 1 if normalized
    if (std::abs(t0 - 1.0) > 1e-6 && t0 > 1e-10) {
        // Normalize if needed
        double t = 1.0 / std::sqrt(t0);
        qw *= t; qx *= t; qy *= t; qz *= t;
        tw2 = qw*qw;
        tx2 = qx*qx;
        ty2 = qy*qy;
        tz2 = qz*qz;
    }
    
    // Calculate coefficients for rotation matrix
    double R11 = tw2 + tx2 - ty2 - tz2; // 1 - 2*(qy²+qz²)
    double R12 = 2.0 * (qx*qy - qw*qz);  // 2*(qx*qy-qw*qz)
    double R13 = 2.0 * (qx*qz + qw*qy);  // 2*(qx*qz+qw*qy)
    double R21 = 2.0 * (qx*qy + qw*qz);  // 2*(qx*qy+qw*qz)
    double R22 = tw2 - tx2 + ty2 - tz2;  // 1 - 2*(qx²+qz²)
    double R23 = 2.0 * (qy*qz - qw*qx);  // 2*(qy*qz-qw*qx) 
    double R31 = 2.0 * (qx*qz - qw*qy);  // 2*(qx*qz-qw*qy)
    double R32 = 2.0 * (qy*qz + qw*qx);  // 2*(qy*qz+qw*qx)
    double R33 = tw2 - tx2 - ty2 + tz2;  // 1 - 2*(qx²+qy²)
    
    // Rotate negated translation (R^T * (-t)) where R^T is transpose of rotation matrix
    // For inverting a rigid transform, we apply inv_R to negated translation
    double rx = R11 * nx + R21 * ny + R31 * nz;
    double ry = R12 * nx + R22 * ny + R32 * nz;
    double rz = R13 * nx + R23 * ny + R33 * nz;
    
    // Store results
    inverted.transform.translation.x = rx;
    inverted.transform.translation.y = ry;
    inverted.transform.translation.z = rz;
    
    inverted.transform.rotation.w = inv_qw;
    inverted.transform.rotation.x = inv_qx;
    inverted.transform.rotation.y = inv_qy;
    inverted.transform.rotation.z = inv_qz;
    
    // Debug output inverted
    std::cout << "Inverted transform:" << std::endl;
    std::cout << "  Frame: " << inverted.header.frame_id << " -> " << inverted.child_frame_id << std::endl;
    std::cout << "  Rotation (w,x,y,z): " << inv_qw << " " << inv_qx << " " << inv_qy << " " << inv_qz << std::endl;
    std::cout << "  Translation (x,y,z): " << rx << " " << ry << " " << rz << std::endl;
    std::cout << "  Rotation coefficients:" << std::endl;
    std::cout << "    R11: " << R11 << " R12: " << R12 << " R13: " << R13 << std::endl;
    std::cout << "    R21: " << R21 << " R22: " << R22 << " R23: " << R23 << std::endl;
    std::cout << "    R31: " << R31 << " R32: " << R32 << " R33: " << R33 << std::endl;
    
    return inverted;
}

geometry_msgs::TransformStamped Buffer::_composeTransforms(
    const geometry_msgs::TransformStamped& t1,
    const geometry_msgs::TransformStamped& t2) {
    
    geometry_msgs::TransformStamped result;
    
    // Set frame IDs for the composed transform
    // t1 connects t1.header.frame_id to t1.child_frame_id
    // t2 connects t2.header.frame_id to t2.child_frame_id
    // The composed transform should connect t1.header.frame_id to t2.child_frame_id
    // Note: For this to work correctly, t1.child_frame_id should equal t2.header.frame_id
    result.header.frame_id = t1.header.frame_id;
    result.child_frame_id = t2.child_frame_id;
    
    // Debug info for frame composition
    std::cout << "\nComposing transforms:" << std::endl;
    std::cout << "  T1 frame: " << t1.header.frame_id << " -> " << t1.child_frame_id << std::endl; 
    std::cout << "  T2 frame: " << t2.header.frame_id << " -> " << t2.child_frame_id << std::endl;
    std::cout << "  Result frame: " << result.header.frame_id << " -> " << result.child_frame_id << std::endl;
    
    // Use the latest timestamp
    if ((t1.header.stamp.sec > t2.header.stamp.sec) ||
        (t1.header.stamp.sec == t2.header.stamp.sec && t1.header.stamp.nsec > t2.header.stamp.nsec)) {
        result.header.stamp = t1.header.stamp;
    } else {
        result.header.stamp = t2.header.stamp;
    }
    
    // Extract quaternion components for both transforms
    double t1w = t1.transform.rotation.w;
    double t1x = t1.transform.rotation.x;
    double t1y = t1.transform.rotation.y;
    double t1z = t1.transform.rotation.z;
    
    double t2w = t2.transform.rotation.w;
    double t2x = t2.transform.rotation.x;
    double t2y = t2.transform.rotation.y;
    double t2z = t2.transform.rotation.z;
    
    // Extract translation components
    double t1_tx = t1.transform.translation.x;
    double t1_ty = t1.transform.translation.y;
    double t1_tz = t1.transform.translation.z;
    
    double t2_tx = t2.transform.translation.x;
    double t2_ty = t2.transform.translation.y;
    double t2_tz = t2.transform.translation.z;
    
    // Debug input details
    std::cout << "  T1 rotation (w,x,y,z): " << t1w << " " << t1x << " " << t1y << " " << t1z << std::endl;
    std::cout << "  T1 translation (x,y,z): " << t1_tx << " " << t1_ty << " " << t1_tz << std::endl;
    std::cout << "  T2 rotation (w,x,y,z): " << t2w << " " << t2x << " " << t2y << " " << t2z << std::endl;
    std::cout << "  T2 translation (x,y,z): " << t2_tx << " " << t2_ty << " " << t2_tz << std::endl;
    
    // First ensure both quaternions are normalized
    double t1_len_sq = t1w*t1w + t1x*t1x + t1y*t1y + t1z*t1z;
    double t2_len_sq = t2w*t2w + t2x*t2x + t2y*t2y + t2z*t2z;
    
    if (std::abs(t1_len_sq - 1.0) > 1e-6 && t1_len_sq > 1e-10) {
        double t1_scale = 1.0 / std::sqrt(t1_len_sq);
        t1w *= t1_scale; t1x *= t1_scale; t1y *= t1_scale; t1z *= t1_scale;
    }
    
    if (std::abs(t2_len_sq - 1.0) > 1e-6 && t2_len_sq > 1e-10) {
        double t2_scale = 1.0 / std::sqrt(t2_len_sq);
        t2w *= t2_scale; t2x *= t2_scale; t2y *= t2_scale; t2z *= t2_scale;
    }
    
    // 1. Generate rotation matrix for t1
    // We'll use this to rotate t2's translation
    double xs = t1x * 2.0;  // Pre-calculated 2*x
    double ys = t1y * 2.0;  // Pre-calculated 2*y
    double zs = t1z * 2.0;  // Pre-calculated 2*z
    
    double wx = t1w * xs;
    double wy = t1w * ys;
    double wz = t1w * zs;
    double xx = t1x * xs;
    double xy = t1x * ys;
    double xz = t1x * zs;
    double yy = t1y * ys;
    double yz = t1y * zs;
    double zz = t1z * zs;
    
    // Components of rotation matrix from t1's quaternion
    double t1_R11 = 1.0 - (yy + zz);
    double t1_R12 = xy - wz;
    double t1_R13 = xz + wy;
    double t1_R21 = xy + wz;
    double t1_R22 = 1.0 - (xx + zz);
    double t1_R23 = yz - wx;
    double t1_R31 = xz - wy;
    double t1_R32 = yz + wx;
    double t1_R33 = 1.0 - (xx + yy);
    
    // Debug rotation matrix for t1
    std::cout << "  T1 Rotation Matrix:" << std::endl;
    std::cout << "    [" << t1_R11 << ", " << t1_R12 << ", " << t1_R13 << "]" << std::endl;
    std::cout << "    [" << t1_R21 << ", " << t1_R22 << ", " << t1_R23 << "]" << std::endl;
    std::cout << "    [" << t1_R31 << ", " << t1_R32 << ", " << t1_R33 << "]" << std::endl;
    
    // 2. Rotate t2's translation using t1's rotation matrix
    double t2_tx_rotated = t1_R11 * t2_tx + t1_R12 * t2_ty + t1_R13 * t2_tz;
    double t2_ty_rotated = t1_R21 * t2_tx + t1_R22 * t2_ty + t1_R23 * t2_tz;
    double t2_tz_rotated = t1_R31 * t2_tx + t1_R32 * t2_ty + t1_R33 * t2_tz;
    
    // 3. Combine rotations by multiplying quaternions
    // result_q = t1_q * t2_q
    result.transform.rotation.w = t1w * t2w - t1x * t2x - t1y * t2y - t1z * t2z;
    result.transform.rotation.x = t1w * t2x + t1x * t2w + t1y * t2z - t1z * t2y;
    result.transform.rotation.y = t1w * t2y - t1x * t2z + t1y * t2w + t1z * t2x;
    result.transform.rotation.z = t1w * t2z + t1x * t2y - t1y * t2x + t1z * t2w;
    
    // Normalize the resulting quaternion
    double result_len_sq = 
        result.transform.rotation.w * result.transform.rotation.w +
        result.transform.rotation.x * result.transform.rotation.x +
        result.transform.rotation.y * result.transform.rotation.y +
        result.transform.rotation.z * result.transform.rotation.z;
    
    if (std::abs(result_len_sq - 1.0) > 1e-6 && result_len_sq > 1e-10) {
        double scale = 1.0 / std::sqrt(result_len_sq);
        result.transform.rotation.w *= scale;
        result.transform.rotation.x *= scale;
        result.transform.rotation.y *= scale;
        result.transform.rotation.z *= scale;
    }
    
    // 4. Calculate final translation
    // This is t1's translation plus t2's translation rotated by t1's rotation
    result.transform.translation.x = t1_tx + t2_tx_rotated;
    result.transform.translation.y = t1_ty + t2_ty_rotated;
    result.transform.translation.z = t1_tz + t2_tz_rotated;
    
    // Debug output for intermediate and final results
    std::cout << "  T2 translation after rotation by T1: " << t2_tx_rotated 
              << " " << t2_ty_rotated << " " << t2_tz_rotated << std::endl;
    std::cout << "  Composed rotation (w,x,y,z): " << result.transform.rotation.w << " " 
              << result.transform.rotation.x << " "
              << result.transform.rotation.y << " "
              << result.transform.rotation.z << std::endl;
    std::cout << "  Final translation (x,y,z): " << result.transform.translation.x << " "
              << result.transform.translation.y << " "
              << result.transform.translation.z << std::endl;
    
    return result;
}

std::chrono::system_clock::time_point Buffer::getValidTimestamp() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Default to current time if no transforms are found
    auto now = std::chrono::system_clock::now();
    
    // Look for a valid timestamp in the dynamic buffer
    for (const auto& parent_map : buffer_) {
        for (const auto& child_map : parent_map.second) {
            const auto& transforms = child_map.second;
            if (!transforms.empty()) {
                // Return the timestamp of the first transform found
                return transforms[0].stamp;
            }
        }
    }
    
    // If no transforms in dynamic buffer, check static buffer
    for (const auto& parent_map : static_buffer_) {
        for (const auto& child_map : parent_map.second) {
            const auto& transforms = child_map.second;
            if (!transforms.empty()) {
                // Return the timestamp of the first transform found
                return transforms[0].stamp;
            }
        }
    }
    
    // If no transforms found at all, return current time
    return now;
}

std::chrono::system_clock::time_point Buffer::getMostRecentTimestamp() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // If we have a valid cached timestamp, return it immediately
    if (cached_timestamp_valid_) {
        return cached_most_recent_timestamp_;
    }
    
    // Cache was invalid, so we need to scan all transforms to find the most recent timestamp
    std::chrono::system_clock::time_point most_recent = std::chrono::system_clock::time_point::min();
    bool found = false;
    
    // Check all transforms in dynamic buffer
    for (const auto& parent_map : buffer_) {
        for (const auto& child_map : parent_map.second) {
            const auto& transforms = child_map.second;
            for (const auto& transform : transforms) {
                if (transform.stamp > most_recent) {
                    most_recent = transform.stamp;
                    found = true;
                }
            }
        }
    }
    
    // Check static transforms too (though these are less likely to have recent timestamps)
    for (const auto& parent_map : static_buffer_) {
        for (const auto& child_map : parent_map.second) {
            const auto& transforms = child_map.second;
            for (const auto& transform : transforms) {
                if (transform.stamp > most_recent) {
                    most_recent = transform.stamp;
                    found = true;
                }
            }
        }
    }
    
    // Update the cache with what we found
    if (found) {
        cached_most_recent_timestamp_ = most_recent;
        cached_timestamp_valid_ = true;
    } else {
        // If no timestamp found, return current time
        return std::chrono::system_clock::now();
    }
    
    return cached_most_recent_timestamp_;
}

} // namespace tf_lcm
