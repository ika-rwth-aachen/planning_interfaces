// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

#pragma once

namespace trajectory_planning_msgs {

namespace trajectory_access {

const std::string kExceptionUnknownStateEntry =
    "Trajectory type with the following ID does not support requested entry: ";

inline int indexT(const unsigned char& type_id) {
  switch (type_id) {
    case DRIVABLE::TYPE_ID:
      return DRIVABLE::T;
    case DRIVABLERWS::TYPE_ID:
      return DRIVABLERWS::T;
    case REFERENCE::TYPE_ID:
      return REFERENCE::T;
    default:
      throw std::invalid_argument(kExceptionUnknownStateEntry + std::to_string(type_id) + ", " + "t");
  }
}

inline int indexX(const unsigned char& type_id) {
  switch (type_id) {
    case DRIVABLE::TYPE_ID:
      return DRIVABLE::X;
    case DRIVABLERWS::TYPE_ID:
      return DRIVABLERWS::X;
    case REFERENCE::TYPE_ID:
      return REFERENCE::X;
    default:
      throw std::invalid_argument(kExceptionUnknownStateEntry + std::to_string(type_id) + ", " + "x");
  }
}

inline int indexY(const unsigned char& type_id) {
  switch (type_id) {
    case DRIVABLE::TYPE_ID:
      return DRIVABLE::Y;
    case DRIVABLERWS::TYPE_ID:
      return DRIVABLERWS::Y;
    case REFERENCE::TYPE_ID:
      return REFERENCE::Y;
    default:
      throw std::invalid_argument(kExceptionUnknownStateEntry + std::to_string(type_id) + ", " + "y");
  }
}

inline int indexV(const unsigned char& type_id) {
  switch (type_id) {
    case DRIVABLE::TYPE_ID:
      return DRIVABLE::V;
    case DRIVABLERWS::TYPE_ID:
      return DRIVABLERWS::V;
    case REFERENCE::TYPE_ID:
      return REFERENCE::V;
    default:
      throw std::invalid_argument(kExceptionUnknownStateEntry + std::to_string(type_id) + ", " + "v");
  }
}

inline int indexTheta(const unsigned char& type_id) {
  switch (type_id) {
    case DRIVABLE::TYPE_ID:
      return DRIVABLE::THETA;
    case DRIVABLERWS::TYPE_ID:
      return DRIVABLERWS::THETA;
    default:
      throw std::invalid_argument(kExceptionUnknownStateEntry + std::to_string(type_id) + ", " + "theta");
  }
}

inline int indexA(const unsigned char& type_id) {
  switch (type_id) {
    case DRIVABLE::TYPE_ID:
      return DRIVABLE::A;
    case DRIVABLERWS::TYPE_ID:
      return DRIVABLERWS::A;
    default:
      throw std::invalid_argument(kExceptionUnknownStateEntry + std::to_string(type_id) + ", " + "a");
  }
}

inline int indexBeta(const unsigned char& type_id) {
  switch (type_id) {
    case DRIVABLERWS::TYPE_ID:
      return DRIVABLERWS::BETA;
    default:
      throw std::invalid_argument(kExceptionUnknownStateEntry + std::to_string(type_id) + ", " + "beta");
  }
}

inline int indexDeltaFront(const unsigned char& type_id) {
  switch (type_id) {
    case DRIVABLERWS::TYPE_ID:
      return DRIVABLERWS::DELTAFRONT;
    default:
      throw std::invalid_argument(kExceptionUnknownStateEntry + std::to_string(type_id) + ", " + "delta_front");
  }
}

inline int indexDeltaRear(const unsigned char& type_id) {
  switch (type_id) {
    case DRIVABLERWS::TYPE_ID:
      return DRIVABLERWS::DELTAREAR;
    default:
      throw std::invalid_argument(kExceptionUnknownStateEntry + std::to_string(type_id) + ", " + "delta_rear");
  }
}

inline int indexDeltaAck(const unsigned char& type_id) {
  switch (type_id) {
    case DRIVABLE::TYPE_ID:
      return DRIVABLE::DELTA;
    default:
      throw std::invalid_argument(kExceptionUnknownStateEntry + std::to_string(type_id) + ", " + "delta");
  }
}

inline int indexS(const unsigned char& type_id) {
  switch (type_id) {
    case DRIVABLE::TYPE_ID:
      return DRIVABLE::S;
    case DRIVABLERWS::TYPE_ID:
      return DRIVABLERWS::S;
    default:
      throw std::invalid_argument(kExceptionUnknownStateEntry + std::to_string(type_id) + ", " + "s");
  }
}

inline bool hasT(const unsigned char& type_id) {
  switch (type_id) {
    case DRIVABLE::TYPE_ID:
      return true;
    case DRIVABLERWS::TYPE_ID:
      return true;
    case REFERENCE::TYPE_ID:
      return true;
    default:
      return false;
  }
}

inline bool hasX(const unsigned char& type_id) {
  switch (type_id) {
    case DRIVABLE::TYPE_ID:
      return true;
    case DRIVABLERWS::TYPE_ID:
      return true;
    case REFERENCE::TYPE_ID:
      return true;
    default:
      return false;
  }
}

inline bool hasY(const unsigned char& type_id) {
  switch (type_id) {
    case DRIVABLE::TYPE_ID:
      return true;
    case DRIVABLERWS::TYPE_ID:
      return true;
    case REFERENCE::TYPE_ID:
      return true;
    default:
      return false;
  }
}

inline bool hasV(const unsigned char& type_id) {
  switch (type_id) {
    case DRIVABLE::TYPE_ID:
      return true;
    case DRIVABLERWS::TYPE_ID:
      return true;
    case REFERENCE::TYPE_ID:
      return true;
    default:
      return false;
  }
}

inline bool hasTheta(const unsigned char& type_id) {
  switch (type_id) {
    case DRIVABLE::TYPE_ID:
      return true;
    case DRIVABLERWS::TYPE_ID:
      return true;
    case REFERENCE::TYPE_ID:
      return false;
    default:
      return false;
  }
}

inline bool hasA(const unsigned char& type_id) {
  switch (type_id) {
    case DRIVABLE::TYPE_ID:
      return true;
    case DRIVABLERWS::TYPE_ID:
      return true;
    case REFERENCE::TYPE_ID:
      return false;
    default:
      return false;
  }
}

inline bool hasBeta(const unsigned char& type_id) {
  switch (type_id) {
    case DRIVABLE::TYPE_ID:
      return false;
    case DRIVABLERWS::TYPE_ID:
      return true;
    case REFERENCE::TYPE_ID:
      return false;
    default:
      return false;
  }
}

inline bool hasDeltaFront(const unsigned char& type_id) {
  switch (type_id) {
    case DRIVABLE::TYPE_ID:
      return false;
    case DRIVABLERWS::TYPE_ID:
      return true;
    case REFERENCE::TYPE_ID:
      return false;
    default:
      return false;
  }
}

inline bool hasDeltaRear(const unsigned char& type_id) {
  switch (type_id) {
    case DRIVABLE::TYPE_ID:
      return false;
    case DRIVABLERWS::TYPE_ID: 
      return true;
    case REFERENCE::TYPE_ID:
      return false;
    default:
      return false;
  }
}

inline bool hasDeltaAck(const unsigned char& type_id) {
  switch (type_id) {
    case DRIVABLE::TYPE_ID:
      return true;
    case DRIVABLERWS::TYPE_ID:
      return false;
    case REFERENCE::TYPE_ID:
      return false;
    default:
      return false;
  }
}

inline bool hasS(const unsigned char& type_id) {
  switch (type_id) {
    case DRIVABLE::TYPE_ID:
      return true;
    case DRIVABLERWS::TYPE_ID:
      return true;
    case REFERENCE::TYPE_ID:
      return false;
    default:
      return false;
  }
}

}  // namespace trajectory_access

}  // namespace trajectory_planning_msgs