#ifndef _JOINT_ADDRESS_HPP_
#define _JOINT_ADDRESS_HPP_
#include <string>
#include <cassert>
namespace mujoco_node {

class IndexRange {
public:
    IndexRange():b_(0), e_(0) {}
    /**
     * @brief Construct a new Index Range object
     * 
     * @param b  开始的下标索引
     * @param e  最后一个下标的索引
     */
    IndexRange(int b, int e) : b_(b), e_(e+1) {
        assert(b >= 0 && e >= b && "Start index must be non-negative and end index must be greater than or equal to start index");
    }

    int size() const {
        return e_ - b_;
    }
    
    void set_range(int b, int e) {
        assert(b >= 0 && e >= b && "Start index must be non-negative and end index must be greater than or equal to start index");
        b_ = b;
        e_ = e + 1;
    }
    
    bool invalid() const {
        return b_ < 0 || e_ <= b_;
    }

    class Iterator {
    public:
        Iterator(int current) : current(current) {}

        int operator*() const {
            return current;
        }

        Iterator& operator++() { // Prefix increment
            ++current;
            return *this;
        }

        Iterator operator++(int) { // Postfix increment
            Iterator temp = *this;
            ++current;
            return temp;
        }

        Iterator& operator--() { // Prefix decrement
            --current;
            return *this;
        }

        Iterator operator--(int) { // Postfix decrement
            Iterator temp = *this;
            --current;
            return temp;
        }

        bool operator!=(const Iterator& other) const {
            return current != other.current;
        }

        // Implicit conversion operator to int
        operator int() const {
            return current;
        }

        friend std::ostream& operator<<(std::ostream& os, const Iterator& ir) {
            os << *ir;
            return os;
        }
    private:
        int current;
    };

    Iterator begin() const {
        return Iterator(b_);
    }

    Iterator end() const {
        return Iterator(e_);
    }
private:
    int b_;
    int e_;
};

class JointGroupAddress {
public:
    explicit JointGroupAddress(const std::string& tag):tag_(tag) {}

    #define SET_GET_ADDRESS(member) \
    JointGroupAddress& set_##member##adr(int b, int e) { \
        this->member##_= IndexRange(b, e); \
        return *this; \
    } \
    const IndexRange& member##adr() const { return this->member##_; }

    // JointGroupAddress& set_qposadr(int b, int e);
    // const IndexRange& qposadr() const;
    SET_GET_ADDRESS(qpos)
    SET_GET_ADDRESS(ctrl)
    SET_GET_ADDRESS(qdof)

    const std::string & tag() const { return tag_; }

    friend std::ostream& operator<<(std::ostream& os, const JointGroupAddress& jga) {
        os << jga.tag_ << ": addr={ "
        << "qpos: [" << jga.qpos_.begin() << ", " << jga.qpos_.end() - 1 << "], "
        << "qdof: [" << jga.qdof_.begin() << ", " << jga.qdof_.end() - 1 << "], "
        << "ctrl: [" << jga.ctrl_.begin() << ", " << jga.ctrl_.end() - 1 << "] "
        << "}";
        return os;
    }

private:
    std::string tag_;
    IndexRange qpos_;  // addr in 'qpos' for joint's data, from mjModel.jnt_qposadr[mj_name2id(mjtObj::mjOBJ_JOINT, name)] 
    IndexRange qdof_;  // addr in 'qvel','qacc',actuator forces  for joint's data, from mjModel.jnt_dofadr[mj_name2id(mjtObj::mjOBJ_JOINT, name)]
    IndexRange ctrl_;  // addr of actuators/controls, from mj_name2id(mnew, mjOBJ_ACTUATOR, motor_name)

    // IndexRange qvel_;  // addr in 'qvel' for joint's data, from mjModel.jnt_dofadr[mj_name2id(mjtObj::mjOBJ_JOINT, name)]
    // IndexRange qacc_;  // addr in 'qacc' for joint's data, from mjModel.jnt_dofadr[mj_name2id(mjtObj::mjOBJ_JOINT, name)] 
    // IndexRange qfrc_;  // addr of actuator forces, from mjModel.jnt_dofadr[mj_name2id(mjtObj::mjOBJ_JOINT, name)] 
};

} // namespace mujoco
#endif