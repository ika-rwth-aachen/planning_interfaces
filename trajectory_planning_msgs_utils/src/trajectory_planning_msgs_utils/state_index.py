# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: MIT

from trajectory_planning_msgs.msg import DRIVABLE, DRIVABLERWS, REFERENCE


kExceptionUnknownStateEntry = (
    "Trajectory type with the following ID does not support requested entry: "
)


def index_t(type_id: int) -> int:
    if type_id == DRIVABLE.TYPE_ID:
        return DRIVABLE.T
    if type_id == DRIVABLERWS.TYPE_ID:
        return DRIVABLERWS.T
    if type_id == REFERENCE.TYPE_ID:
        return REFERENCE.T
    raise ValueError(kExceptionUnknownStateEntry + f"{type_id}, t")


def index_x(type_id: int) -> int:
    if type_id == DRIVABLE.TYPE_ID:
        return DRIVABLE.X
    if type_id == DRIVABLERWS.TYPE_ID:
        return DRIVABLERWS.X
    if type_id == REFERENCE.TYPE_ID:
        return REFERENCE.X
    raise ValueError(kExceptionUnknownStateEntry + f"{type_id}, x")


def index_y(type_id: int) -> int:
    if type_id == DRIVABLE.TYPE_ID:
        return DRIVABLE.Y
    if type_id == DRIVABLERWS.TYPE_ID:
        return DRIVABLERWS.Y
    if type_id == REFERENCE.TYPE_ID:
        return REFERENCE.Y
    raise ValueError(kExceptionUnknownStateEntry + f"{type_id}, y")


def index_v(type_id: int) -> int:
    if type_id == DRIVABLE.TYPE_ID:
        return DRIVABLE.V
    if type_id == DRIVABLERWS.TYPE_ID:
        return DRIVABLERWS.V
    if type_id == REFERENCE.TYPE_ID:
        return REFERENCE.V
    raise ValueError(kExceptionUnknownStateEntry + f"{type_id}, v")


def index_theta(type_id: int) -> int:
    if type_id == DRIVABLE.TYPE_ID:
        return DRIVABLE.THETA
    if type_id == DRIVABLERWS.TYPE_ID:
        return DRIVABLERWS.THETA
    raise ValueError(kExceptionUnknownStateEntry + f"{type_id}, theta")


def index_a(type_id: int) -> int:
    if type_id == DRIVABLE.TYPE_ID:
        return DRIVABLE.A
    if type_id == DRIVABLERWS.TYPE_ID:
        return DRIVABLERWS.A
    raise ValueError(kExceptionUnknownStateEntry + f"{type_id}, a")


def index_beta(type_id: int) -> int:
    if type_id == DRIVABLERWS.TYPE_ID:
        return DRIVABLERWS.BETA
    raise ValueError(kExceptionUnknownStateEntry + f"{type_id}, beta")


def index_delta_front(type_id: int) -> int:
    if type_id == DRIVABLERWS.TYPE_ID:
        return DRIVABLERWS.DELTAFRONT
    raise ValueError(kExceptionUnknownStateEntry + f"{type_id}, delta_front")


def index_delta_rear(type_id: int) -> int:
    if type_id == DRIVABLERWS.TYPE_ID:
        return DRIVABLERWS.DELTAREAR
    raise ValueError(kExceptionUnknownStateEntry + f"{type_id}, delta_rear")


def index_delta_ack(type_id: int) -> int:
    if type_id == DRIVABLE.TYPE_ID:
        return DRIVABLE.DELTA
    raise ValueError(kExceptionUnknownStateEntry + f"{type_id}, delta")


def index_s(type_id: int) -> int:
    if type_id == DRIVABLE.TYPE_ID:
        return DRIVABLE.S
    if type_id == DRIVABLERWS.TYPE_ID:
        return DRIVABLERWS.S
    raise ValueError(kExceptionUnknownStateEntry + f"{type_id}, s")


def has_t(type_id: int) -> bool:
    return type_id in {DRIVABLE.TYPE_ID, DRIVABLERWS.TYPE_ID, REFERENCE.TYPE_ID}


def has_x(type_id: int) -> bool:
    return type_id in {DRIVABLE.TYPE_ID, DRIVABLERWS.TYPE_ID, REFERENCE.TYPE_ID}


def has_y(type_id: int) -> bool:
    return type_id in {DRIVABLE.TYPE_ID, DRIVABLERWS.TYPE_ID, REFERENCE.TYPE_ID}


def has_v(type_id: int) -> bool:
    return type_id in {DRIVABLE.TYPE_ID, DRIVABLERWS.TYPE_ID, REFERENCE.TYPE_ID}


def has_theta(type_id: int) -> bool:
    return type_id in {DRIVABLE.TYPE_ID, DRIVABLERWS.TYPE_ID}


def has_a(type_id: int) -> bool:
    return type_id in {DRIVABLE.TYPE_ID, DRIVABLERWS.TYPE_ID}


def has_beta(type_id: int) -> bool:
    return type_id == DRIVABLERWS.TYPE_ID


def has_delta_front(type_id: int) -> bool:
    return type_id == DRIVABLERWS.TYPE_ID


def has_delta_rear(type_id: int) -> bool:
    return type_id == DRIVABLERWS.TYPE_ID


def has_delta_ack(type_id: int) -> bool:
    return type_id == DRIVABLE.TYPE_ID


def has_s(type_id: int) -> bool:
    return type_id in {DRIVABLE.TYPE_ID, DRIVABLERWS.TYPE_ID}
