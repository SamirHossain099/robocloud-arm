import time

import speech_recognition as sr

from pi.controller.arm import Arm
from pi.controller.executor import CommandRouter
from pi.logutil import vprint


def voice_control(router: CommandRouter) -> None:
    r = sr.Recognizer()
    mic = sr.Microphone(device_index=0)

    vprint("Voice ready (say: dummy <command>)")

    while True:
        try:
            # Keep source open to avoid repeatedly reopening ALSA stream.
            with mic as source:
                r.adjust_for_ambient_noise(source, duration=0.5)
                while True:
                    try:
                        audio = r.listen(source, timeout=1, phrase_time_limit=3)
                    except sr.WaitTimeoutError:
                        continue

                    try:
                        text = r.recognize_google(audio).lower()
                    except sr.UnknownValueError:
                        continue
                    except sr.RequestError as err:
                        vprint(f"Voice recognition request failed: {err}")
                        time.sleep(1.0)
                        continue

                    vprint("Heard:", text)

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
        except Exception as err:
            # Surface setup errors (bad mic index / missing capture device) instead of silent failure.
            vprint(f"Voice input error: {err}")
            time.sleep(2.0)
