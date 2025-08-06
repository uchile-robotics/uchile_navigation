#!/usr/bin/env python3
import rclpy
from uchile_navigation.nav_server import NavigationSkill
import time

def main():
    rclpy.init()

    nav = NavigationSkill()

    # Esperar a que se localice
    timeout = 10  # segundos
    start_time = time.time()
    while not nav.is_localized() and (time.time() - start_time) < timeout:
        rclpy.spin_once(nav, timeout_sec=0.1)

    if not nav.is_localized():
        nav.get_logger().error("Robot no está localizado. Abortando prueba.")
        nav.destroy_node()
        rclpy.shutdown()
        return

    nav.get_logger().info("Robot localizado. Enviando objetivo...")

    success = nav.go_to_point(x=1.0, y=2.0, theta=0.0)
    if not success:
        nav.get_logger().error("No se pudo enviar el objetivo.")
        nav.destroy_node()
        rclpy.shutdown()
        return

    # Esperar resultado (bloqueante)
    result = nav.wait_for_result(timeout=60.0)

    if result is True:
        nav.get_logger().info("Objetivo alcanzado correctamente.")
    elif result is False:
        nav.get_logger().warn("El robot falló en alcanzar el objetivo.")
    else:
        nav.get_logger().warn("Tiempo de espera agotado para alcanzar el objetivo.")

    nav.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
