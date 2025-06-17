#include <iostream>
#include <string>
#include <cmath>
#include <lcm_msgs/geometry_msgs/TransformStamped.hpp>
#include "buffer.h"

// Function to print a transform for debugging
void printTransform(const std::string& prefix, const geometry_msgs::TransformStamped& transform) {
    std::cout << prefix << " Frame: " << transform.header.frame_id << " -> " << transform.child_frame_id << std::endl;
    std::cout << prefix << " Rotation (w,x,y,z): " 
              << transform.transform.rotation.w << ", " 
              << transform.transform.rotation.x << ", " 
              << transform.transform.rotation.y << ", " 
              << transform.transform.rotation.z << std::endl;
    std::cout << prefix << " Translation (x,y,z): " 
              << transform.transform.translation.x << ", " 
              << transform.transform.translation.y << ", " 
              << transform.transform.translation.z << std::endl;
}

// Function to compare two transforms and print differences
bool compareTransforms(const geometry_msgs::TransformStamped& t1, 
                     const geometry_msgs::TransformStamped& t2,
                     double epsilon = 1e-6) {
    bool equal = true;
    
    // Compare rotation quaternions
    if (std::abs(t1.transform.rotation.w - t2.transform.rotation.w) > epsilon ||
        std::abs(t1.transform.rotation.x - t2.transform.rotation.x) > epsilon ||
        std::abs(t1.transform.rotation.y - t2.transform.rotation.y) > epsilon ||
        std::abs(t1.transform.rotation.z - t2.transform.rotation.z) > epsilon) {
        
        std::cout << "Rotation mismatch:" << std::endl;
        std::cout << "  First:  (w,x,y,z) = (" 
                 << t1.transform.rotation.w << ", " 
                 << t1.transform.rotation.x << ", " 
                 << t1.transform.rotation.y << ", " 
                 << t1.transform.rotation.z << ")" << std::endl;
        std::cout << "  Second: (w,x,y,z) = (" 
                 << t2.transform.rotation.w << ", " 
                 << t2.transform.rotation.x << ", " 
                 << t2.transform.rotation.y << ", " 
                 << t2.transform.rotation.z << ")" << std::endl;
        std::cout << "  Difference: (" 
                 << t1.transform.rotation.w - t2.transform.rotation.w << ", " 
                 << t1.transform.rotation.x - t2.transform.rotation.x << ", " 
                 << t1.transform.rotation.y - t2.transform.rotation.y << ", " 
                 << t1.transform.rotation.z - t2.transform.rotation.z << ")" << std::endl;
        equal = false;
    }
    
    // Compare translation vectors
    if (std::abs(t1.transform.translation.x - t2.transform.translation.x) > epsilon ||
        std::abs(t1.transform.translation.y - t2.transform.translation.y) > epsilon ||
        std::abs(t1.transform.translation.z - t2.transform.translation.z) > epsilon) {
        
        std::cout << "Translation mismatch:" << std::endl;
        std::cout << "  First:  (x,y,z) = (" 
                 << t1.transform.translation.x << ", " 
                 << t1.transform.translation.y << ", " 
                 << t1.transform.translation.z << ")" << std::endl;
        std::cout << "  Second: (x,y,z) = (" 
                 << t2.transform.translation.x << ", " 
                 << t2.transform.translation.y << ", " 
                 << t2.transform.translation.z << ")" << std::endl;
        std::cout << "  Difference: (" 
                 << t1.transform.translation.x - t2.transform.translation.x << ", " 
                 << t1.transform.translation.y - t2.transform.translation.y << ", " 
                 << t1.transform.translation.z - t2.transform.translation.z << ")" << std::endl;
        equal = false;
    }
    
    return equal;
}

// Function to verify that a transform and its inverse compose to identity
bool verifyInverseConsistency(Buffer& buffer, 
                            const geometry_msgs::TransformStamped& transform, 
                            double epsilon = 1e-6) {
    std::cout << "\n--- Testing inverse consistency ---" << std::endl;
    
    // Get the inverse transform
    geometry_msgs::TransformStamped inverse = buffer._inverseTransform(transform);
    
    // Now compose them: transform * inverse_transform (should be identity)
    geometry_msgs::TransformStamped composed_t_tinv = buffer._composeTransforms(transform, inverse);
    
    // And inverse * transform (should also be identity)
    geometry_msgs::TransformStamped composed_tinv_t = buffer._composeTransforms(inverse, transform);
    
    // Check if the composition approximates identity transform
    bool identity_check1 = (std::abs(composed_t_tinv.transform.rotation.w - 1.0) < epsilon &&
                          std::abs(composed_t_tinv.transform.rotation.x) < epsilon &&
                          std::abs(composed_t_tinv.transform.rotation.y) < epsilon &&
                          std::abs(composed_t_tinv.transform.rotation.z) < epsilon &&
                          std::abs(composed_t_tinv.transform.translation.x) < epsilon &&
                          std::abs(composed_t_tinv.transform.translation.y) < epsilon &&
                          std::abs(composed_t_tinv.transform.translation.z) < epsilon);
    
    bool identity_check2 = (std::abs(composed_tinv_t.transform.rotation.w - 1.0) < epsilon &&
                          std::abs(composed_tinv_t.transform.rotation.x) < epsilon &&
                          std::abs(composed_tinv_t.transform.rotation.y) < epsilon &&
                          std::abs(composed_tinv_t.transform.rotation.z) < epsilon &&
                          std::abs(composed_tinv_t.transform.translation.x) < epsilon &&
                          std::abs(composed_tinv_t.transform.translation.y) < epsilon &&
                          std::abs(composed_tinv_t.transform.translation.z) < epsilon);
    
    std::cout << "Transform * Inverse results:" << std::endl;
    printTransform("  ", composed_t_tinv);
    std::cout << "  Identity check: " << (identity_check1 ? "PASSED" : "FAILED") << std::endl;
    
    std::cout << "Inverse * Transform results:" << std::endl;
    printTransform("  ", composed_tinv_t);
    std::cout << "  Identity check: " << (identity_check2 ? "PASSED" : "FAILED") << std::endl;
    
    return identity_check1 && identity_check2;
}

// Function to test double inversion (should get original transform back)
bool verifyDoubleInversion(Buffer& buffer, 
                         const geometry_msgs::TransformStamped& transform,
                         double epsilon = 1e-6) {
    std::cout << "\n--- Testing double inversion ---" << std::endl;
    
    // Invert twice
    geometry_msgs::TransformStamped inverse = buffer._inverseTransform(transform);
    geometry_msgs::TransformStamped double_inverse = buffer._inverseTransform(inverse);
    
    // Compare original and double inverse
    std::cout << "Original transform:" << std::endl;
    printTransform("  ", transform);
    
    std::cout << "Double inverted transform:" << std::endl;
    printTransform("  ", double_inverse);
    
    bool result = compareTransforms(transform, double_inverse, epsilon);
    std::cout << "Double inversion check: " << (result ? "PASSED" : "FAILED") << std::endl;
    
    return result;
}

// Function to verify that composed transforms are consistent
bool verifyCompositionAssociativity(Buffer& buffer, 
                                 const geometry_msgs::TransformStamped& t1,
                                 const geometry_msgs::TransformStamped& t2,
                                 const geometry_msgs::TransformStamped& t3,
                                 double epsilon = 1e-6) {
    std::cout << "\n--- Testing composition associativity ---" << std::endl;
    
    // Compute (t1 * t2) * t3
    geometry_msgs::TransformStamped t1_t2 = buffer._composeTransforms(t1, t2);
    geometry_msgs::TransformStamped t1_t2_t3 = buffer._composeTransforms(t1_t2, t3);
    
    // Compute t1 * (t2 * t3)
    geometry_msgs::TransformStamped t2_t3 = buffer._composeTransforms(t2, t3);
    geometry_msgs::TransformStamped t1_t2_t3_alt = buffer._composeTransforms(t1, t2_t3);
    
    // Compare both computations
    std::cout << "Composition (t1*t2)*t3:" << std::endl;
    printTransform("  ", t1_t2_t3);
    
    std::cout << "Composition t1*(t2*t3):" << std::endl;
    printTransform("  ", t1_t2_t3_alt);
    
    bool result = compareTransforms(t1_t2_t3, t1_t2_t3_alt, epsilon);
    std::cout << "Associativity check: " << (result ? "PASSED" : "FAILED") << std::endl;
    
    return result;
}

// Create a standard transform for testing
geometry_msgs::TransformStamped createTestTransform(
    const std::string& frame_id,
    const std::string& child_frame_id,
    double tx, double ty, double tz,
    double qw, double qx, double qy, double qz) {
    
    geometry_msgs::TransformStamped transform;
    transform.header.frame_id = frame_id;
    transform.child_frame_id = child_frame_id;
    
    transform.transform.translation.x = tx;
    transform.transform.translation.y = ty;
    transform.transform.translation.z = tz;
    
    // Normalize quaternion
    double mag = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    
    transform.transform.rotation.w = qw / mag;
    transform.transform.rotation.x = qx / mag;
    transform.transform.rotation.y = qy / mag;
    transform.transform.rotation.z = qz / mag;
    
    return transform;
}

int main() {
    // Create a Buffer instance for testing
    Buffer buffer;
    bool all_tests_passed = true;
    
    std::cout << "===== Testing tf_lcm transform operations =====" << std::endl;
    
    // Test case 1: Simple transform
    geometry_msgs::TransformStamped simple_transform = createTestTransform(
        "world", "camera", 
        1.0, 2.0, 3.0,       // translation
        1.0, 0.0, 0.0, 0.0  // rotation (identity quaternion)
    );
    std::cout << "\nTest Case 1: Simple transform with identity rotation" << std::endl;
    printTransform("Original:", simple_transform);
    all_tests_passed &= verifyInverseConsistency(buffer, simple_transform);
    all_tests_passed &= verifyDoubleInversion(buffer, simple_transform);
    
    // Test case 2: Transform with 90-degree rotation around X
    geometry_msgs::TransformStamped rot_x_transform = createTestTransform(
        "world", "camera", 
        1.0, 2.0, 3.0,                // translation
        0.7071, 0.7071, 0.0, 0.0      // rotation (90° around X)
    );
    std::cout << "\nTest Case 2: 90° rotation around X axis" << std::endl;
    printTransform("Original:", rot_x_transform);
    all_tests_passed &= verifyInverseConsistency(buffer, rot_x_transform);
    all_tests_passed &= verifyDoubleInversion(buffer, rot_x_transform);
    
    // Test case 3: Transform with 90-degree rotation around Y
    geometry_msgs::TransformStamped rot_y_transform = createTestTransform(
        "world", "camera", 
        1.0, 2.0, 3.0,                // translation
        0.7071, 0.0, 0.7071, 0.0      // rotation (90° around Y)
    );
    std::cout << "\nTest Case 3: 90° rotation around Y axis" << std::endl;
    printTransform("Original:", rot_y_transform);
    all_tests_passed &= verifyInverseConsistency(buffer, rot_y_transform);
    all_tests_passed &= verifyDoubleInversion(buffer, rot_y_transform);
    
    // Test case 4: Transform with 90-degree rotation around Z
    geometry_msgs::TransformStamped rot_z_transform = createTestTransform(
        "world", "camera", 
        1.0, 2.0, 3.0,                // translation
        0.7071, 0.0, 0.0, 0.7071      // rotation (90° around Z)
    );
    std::cout << "\nTest Case 4: 90° rotation around Z axis" << std::endl;
    printTransform("Original:", rot_z_transform);
    all_tests_passed &= verifyInverseConsistency(buffer, rot_z_transform);
    all_tests_passed &= verifyDoubleInversion(buffer, rot_z_transform);
    
    // Test case 5: Complex rotation
    geometry_msgs::TransformStamped complex_transform = createTestTransform(
        "world", "camera", 
        1.95, -0.5, 2.3,              // translation
        0.5, 0.5, 0.5, 0.5            // rotation (120° around [1,1,1])
    );
    std::cout << "\nTest Case 5: Complex rotation" << std::endl;
    printTransform("Original:", complex_transform);
    all_tests_passed &= verifyInverseConsistency(buffer, complex_transform);
    all_tests_passed &= verifyDoubleInversion(buffer, complex_transform);
    
    // Test composition with multiple transforms
    std::cout << "\n===== Testing transform composition =====" << std::endl;
    
    // Create a chain of three transforms for testing
    geometry_msgs::TransformStamped world_to_base = createTestTransform(
        "world", "base", 
        0.0, 0.0, 1.0,         // translation
        0.7071, 0.0, 0.7071, 0.0  // rotation (90° around Y)
    );
    
    geometry_msgs::TransformStamped base_to_arm = createTestTransform(
        "base", "arm", 
        1.0, 0.0, 0.5,         // translation
        0.7071, 0.0, 0.0, 0.7071  // rotation (90° around Z)
    );
    
    geometry_msgs::TransformStamped arm_to_camera = createTestTransform(
        "arm", "camera", 
        0.2, 0.1, 0.3,         // translation
        0.7071, 0.7071, 0.0, 0.0  // rotation (90° around X)
    );
    
    // Test composition associativity
    all_tests_passed &= verifyCompositionAssociativity(
        buffer, world_to_base, base_to_arm, arm_to_camera
    );
    
    // Final report
    std::cout << "\n===== Test Summary =====" << std::endl;
    std::cout << "All tests " << (all_tests_passed ? "PASSED!" : "FAILED!") << std::endl;
    
    return all_tests_passed ? 0 : 1;
}
