
import math
import struct
import time

import cv2
import numpy as np
import serial
from serial import SerialException


SERIAL_PORT = "COM10"
BAUD_RATE = 230400

WINDOW_SIZE = 800
MAX_DISTANCE_MM = 4000
MIN_CONFIDENCE = 10
POINT_LIFETIME_SEC = 1.0
ANGLE_OFFSET_DEG = 0.0

HEADER = 0x54
VER_LEN = 0x2C
PACKET_SIZE = 47
POINT_COUNT = 12


class LD06Reader:
    def __init__(self, port: str, baud_rate: int) -> None:
        self.serial = serial.Serial(
            port=port,
            baudrate=baud_rate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0,
        )

        self.buffer = bytearray()
        self.packet_count = 0

    def close(self) -> None:
        if self.serial.is_open:
            self.serial.close()

    def read_packets(self) -> list[bytes]:
        waiting = self.serial.in_waiting

        if waiting > 0:
            self.buffer.extend(self.serial.read(waiting))

        packets: list[bytes] = []

        while True:
            if len(self.buffer) < 2:
                break

            header_index = self.buffer.find(bytes([HEADER]))

            if header_index < 0:
                self.buffer.clear()
                break

            if header_index > 0:
                del self.buffer[:header_index]

            if len(self.buffer) < PACKET_SIZE:
                break

            if self.buffer[1] != VER_LEN:
                del self.buffer[0]
                continue

            packet = bytes(self.buffer[:PACKET_SIZE])
            del self.buffer[:PACKET_SIZE]

            packets.append(packet)
            self.packet_count += 1

        return packets


def parse_packet(
    packet: bytes,
) -> tuple[list[tuple[float, int, int]], float] | None:
    if len(packet) != PACKET_SIZE:
        return None

    if packet[0] != HEADER:
        return None

    if packet[1] != VER_LEN:
        return None

    speed_deg_per_sec = struct.unpack_from("<H", packet, 2)[0]

    start_angle_deg = (
        struct.unpack_from("<H", packet, 4)[0] / 100.0
    )

    end_angle_offset = 6 + POINT_COUNT * 3

    end_angle_deg = (
        struct.unpack_from(
            "<H",
            packet,
            end_angle_offset,
        )[0]
        / 100.0
    )

    angle_difference = end_angle_deg - start_angle_deg

    if angle_difference < 0:
        angle_difference += 360.0

    points: list[tuple[float, int, int]] = []

    for index in range(POINT_COUNT):
        data_offset = 6 + index * 3

        distance_mm = struct.unpack_from(
            "<H",
            packet,
            data_offset,
        )[0]

        confidence = packet[data_offset + 2]

        angle_deg = (
            start_angle_deg
            + angle_difference
            * index
            / (POINT_COUNT - 1)
        ) % 360.0

        angle_deg = (
            angle_deg + ANGLE_OFFSET_DEG
        ) % 360.0

        points.append(
            (
                angle_deg,
                distance_mm,
                confidence,
            )
        )

    return points, float(speed_deg_per_sec)


def draw_grid(
    image: np.ndarray,
    center: tuple[int, int],
    maximum_radius: int,
) -> None:
    cx, cy = center

    for distance_mm in range(
        1000,
        MAX_DISTANCE_MM + 1,
        1000,
    ):
        radius = int(
            distance_mm
            / MAX_DISTANCE_MM
            * maximum_radius
        )

        cv2.circle(
            image,
            center,
            radius,
            (70, 70, 70),
            1,
            cv2.LINE_AA,
        )

        cv2.putText(
            image,
            f"{distance_mm // 1000} m",
            (cx + 5, cy - radius + 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (180, 180, 180),
            1,
            cv2.LINE_AA,
        )

    cv2.line(
        image,
        (cx, cy - maximum_radius),
        (cx, cy + maximum_radius),
        (70, 70, 70),
        1,
        cv2.LINE_AA,
    )

    cv2.line(
        image,
        (cx - maximum_radius, cy),
        (cx + maximum_radius, cy),
        (70, 70, 70),
        1,
        cv2.LINE_AA,
    )

    cv2.putText(
        image,
        "FRONT",
        (cx - 35, cy - maximum_radius - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )


def main() -> None:
    try:
        lidar = LD06Reader(
            SERIAL_PORT,
            BAUD_RATE,
        )

    except SerialException as error:
        print(f"{SERIAL_PORT}を開けませんでした。")
        print(error)
        return

    print(f"{SERIAL_PORT}を開きました。")
    print("Q、ESC、ウィンドウの×、Ctrl+Cで終了します。")

    time.sleep(2.0)
    lidar.serial.reset_input_buffer()

    point_map: dict[
        int,
        tuple[float, int, int, float]
    ] = {}

    speed_deg_per_sec = 0.0

    window_name = "LD06 LiDAR"

    cv2.namedWindow(
        window_name,
        cv2.WINDOW_NORMAL,
    )

    cv2.resizeWindow(
        window_name,
        WINDOW_SIZE,
        WINDOW_SIZE,
    )

    running = True

    try:
        while running:
            packets = lidar.read_packets()
            current_time = time.monotonic()

            for packet in packets:
                result = parse_packet(packet)

                if result is None:
                    continue

                points, speed_deg_per_sec = result

                for (
                    angle_deg,
                    distance_mm,
                    confidence,
                ) in points:
                    angle_index = (
                        int(round(angle_deg)) % 360
                    )

                    point_map[angle_index] = (
                        angle_deg,
                        distance_mm,
                        confidence,
                        current_time,
                    )

            image = np.zeros(
                (
                    WINDOW_SIZE,
                    WINDOW_SIZE,
                    3,
                ),
                dtype=np.uint8,
            )

            center = (
                WINDOW_SIZE // 2,
                WINDOW_SIZE // 2,
            )

            maximum_radius = (
                WINDOW_SIZE // 2 - 55
            )

            draw_grid(
                image,
                center,
                maximum_radius,
            )

            cx, cy = center
            displayed_points = 0

            expired_angles: list[int] = []

            for angle_index, point_data in point_map.items():
                (
                    angle_deg,
                    distance_mm,
                    confidence,
                    updated_time,
                ) = point_data

                if (
                    current_time - updated_time
                    > POINT_LIFETIME_SEC
                ):
                    expired_angles.append(angle_index)
                    continue

                if distance_mm <= 0:
                    continue

                if distance_mm > MAX_DISTANCE_MM:
                    continue

                if confidence < MIN_CONFIDENCE:
                    continue

                radius = (
                    distance_mm
                    / MAX_DISTANCE_MM
                    * maximum_radius
                )

                angle_rad = math.radians(angle_deg)

                x = int(
                    cx
                    + radius
                    * math.sin(angle_rad)
                )

                y = int(
                    cy
                    - radius
                    * math.cos(angle_rad)
                )

                cv2.circle(
                    image,
                    (x, y),
                    2,
                    (0, 255, 0),
                    -1,
                    cv2.LINE_AA,
                )

                displayed_points += 1

            for angle_index in expired_angles:
                del point_map[angle_index]

            cv2.circle(
                image,
                center,
                7,
                (0, 0, 255),
                -1,
                cv2.LINE_AA,
            )

            rpm = speed_deg_per_sec / 6.0

            cv2.putText(
                image,
                f"Speed: {rpm:.1f} rpm",
                (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

            cv2.putText(
                image,
                f"Points: {displayed_points}",
                (20, 58),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

            cv2.putText(
                image,
                f"Packets: {lidar.packet_count}",
                (20, 86),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

            cv2.putText(
                image,
                "Q / ESC / X : Exit",
                (20, WINDOW_SIZE - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (180, 180, 180),
                1,
                cv2.LINE_AA,
            )

            cv2.imshow(
                window_name,
                image,
            )

            key = cv2.waitKeyEx(1)

            if key in (
                ord("q"),
                ord("Q"),
                27,
            ):
                running = False

            try:
                window_visible = cv2.getWindowProperty(
                    window_name,
                    cv2.WND_PROP_VISIBLE,
                )

                if window_visible < 1:
                    running = False

            except cv2.error:
                running = False

    except KeyboardInterrupt:
        pass

    finally:
        lidar.close()
        cv2.destroyAllWindows()
        cv2.waitKey(1)


if __name__ == "__main__":
    main()

