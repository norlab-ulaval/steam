//////////////////////////////////////////////////////////////////////////////////////////////
/// \file StateVariableBase.hpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#ifndef STEAM_STATE_VARIABLE_BASE_HPP
#define STEAM_STATE_VARIABLE_BASE_HPP

#include <mutex>
#include <thread>

#include <Eigen/Core>

namespace steam {

/// Defines type we will use for IDs (to be used in some maps and unordered maps)
typedef unsigned int StateID;

/////////////////////////////////////////////////////////////////////////////////////////////
/// \brief A generator class for IDs
/////////////////////////////////////////////////////////////////////////////////////////////
class IdGenerator
{
 public:

  /////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Static method to generate the next available ID number (thread safe)
  /////////////////////////////////////////////////////////////////////////////////////////////
  static StateID getNextId() {
    static StateID nextId = 0;
    static std::mutex idMutex;
    std::lock_guard<std::mutex> guard(idMutex);
    return ++nextId;
  }
};

/////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Class for managing IDs (allows future extension to 'key' attributes)
/////////////////////////////////////////////////////////////////////////////////////////////
class StateKey
{
public:

  /////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Default constructor (gets next free ID)
  /////////////////////////////////////////////////////////////////////////////////////////////
  StateKey() : id_(IdGenerator::getNextId()) {}

  /////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Get ID of the key
  /////////////////////////////////////////////////////////////////////////////////////////////
  StateID getID() const {return id_;}

  /////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Checks if two keys are equal (this vs other)
  /////////////////////////////////////////////////////////////////////////////////////////////
  bool equals(const StateKey& other) const {return id_ == other.getID();}

private:

  /////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Identification number
  /////////////////////////////////////////////////////////////////////////////////////////////
  StateID id_;

};

/////////////////////////////////////////////////////////////////////////////////////////////
/// \brief State variable interface
/////////////////////////////////////////////////////////////////////////////////////////////
class StateVariableBase
{
public:

  /// Convenience typedefs
  typedef std::shared_ptr<StateVariableBase> Ptr;
  typedef std::shared_ptr<const StateVariableBase> ConstPtr;

  /////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Constructor
  /////////////////////////////////////////////////////////////////////////////////////////////
  StateVariableBase(unsigned int perturbDim, bool isLocked = false)
    : perturbDim_(perturbDim), isLocked_(isLocked) {

    // Throw logic error
    if (perturbDim_ <= 0) {
      throw std::invalid_argument("Tried to initialize a StateVariableBase with "
                                  "a zero-size perturbation dimension");
    }
  }

  /////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Interface to update a state from a perturbation
  /////////////////////////////////////////////////////////////////////////////////////////////
  virtual bool update(const Eigen::VectorXd& perturbation) = 0;

  /////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Interface to set the state value from another instance of the state
  /////////////////////////////////////////////////////////////////////////////////////////////
  virtual void setFromCopy(const ConstPtr& other) = 0;

  /////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Get the states unique key
  /////////////////////////////////////////////////////////////////////////////////////////////
  StateKey getKey() const {
    return key_;
  }

  /////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Get the perturbation dimension
  /////////////////////////////////////////////////////////////////////////////////////////////
  unsigned int getPerturbDim() const {
    return perturbDim_;
  }

  /////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Set the lock state of the state variable (cannot be updated if locked).
  ///        Note: cost terms may reference locked variables, but locked variables should not
  ///        be added to the optimization problem.
  /////////////////////////////////////////////////////////////////////////////////////////////
  void setLock(bool lockState) {
    isLocked_ = lockState;
  }

  /////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Query whether or not the state variable is locked
  /////////////////////////////////////////////////////////////////////////////////////////////
  bool isLocked() const {
    return isLocked_;
  }

  /////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Interface for clone method
  /////////////////////////////////////////////////////////////////////////////////////////////
  virtual Ptr clone() const = 0;

private:

  /////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Unique identifier key, set on construction
  /////////////////////////////////////////////////////////////////////////////////////////////
  const StateKey key_;

  /////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Dimension of the perturbation vector
  /////////////////////////////////////////////////////////////////////////////////////////////
  const unsigned int perturbDim_;

  /////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Whether or not the state variable is locked
  /////////////////////////////////////////////////////////////////////////////////////////////
  bool isLocked_;

};

} // steam

#endif // STEAM_STATE_VARIABLE_BASE_HPP
