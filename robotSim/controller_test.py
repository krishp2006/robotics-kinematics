import pygame

pygame.init()
pygame.joystick.init()

print("Controllers found:", pygame.joystick.get_count())

if pygame.joystick.get_count() > 0:

    controller = pygame.joystick.Joystick(0)
    controller.init()

    print("Controller:", controller.get_name())
    print("Axes:", controller.get_numaxes())
    print("Buttons:", controller.get_numbuttons())

    while True:
        pygame.event.pump()

        print(
            "Left:",
            controller.get_axis(0),
            controller.get_axis(1),
            "Right:",
            controller.get_axis(3),
            controller.get_axis(4)
        )

else:
    print("No controller detected.")

pygame.quit()