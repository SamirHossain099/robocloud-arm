import speech_recognition as sr

from pi.controller.arm import Arm
from pi.controller.executor import CommandRouter


def voice_control(router: CommandRouter) -> None:
    r = sr.Recognizer()
    mic = sr.Microphone(device_index=0)

    print("Voice ready (say: dummy <command>)")

    while True:
        try:
            with mic as source:
                r.adjust_for_ambient_noise(source, duration=0.5)
                audio = r.listen(source)

            text = r.recognize_google(audio).lower()
            print("Heard:", text)

            if "dummy" not in text:
                continue

            if "reset" in text:
                router.submit(Arm.command("reset", {}, "high"))
            elif "claw close" in text or "close claw" in text:
                router.submit(Arm.command("claw_close", {}, "high"))
            elif "claw open" in text or "open claw" in text:
                router.submit(Arm.command("claw_open", {}, "high"))
            else:
                Arm.speak("I did not understand")
        except Exception:
            pass  # ignore noise / errors
