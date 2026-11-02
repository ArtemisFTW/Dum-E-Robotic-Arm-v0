"""Build the complete test firmware with the installed STM32CubeIDE ARM tools.

Run: python build_firmware.py
Output: Build_serial_test/servo_sweep_test.{elf,hex,bin}
This builds only; it never connects to or flashes a board.
"""
from pathlib import Path
import re
import subprocess
import sys


def verify_uart_setup(project):
    """Catch a lost CubeMX interrupt configuration before producing firmware."""
    settings = dict(line.split("=", 1) for line in
                    (project / "servo_sweep_test.ioc").read_text().splitlines()
                    if "=" in line and not line.startswith("#"))
    errors = []
    if not settings.get("NVIC.USART2_IRQn", "").startswith("true"):
        errors.append("Enable USART2 global interrupt in the .ioc NVIC settings.")
    msp = (project / "Core/Src/stm32f4xx_hal_msp.c").read_text()
    if not re.search(r"HAL_NVIC_EnableIRQ\s*\(\s*USART2_IRQn\s*\)", msp):
        errors.append("USART2_IRQn is not enabled in the generated MSP code.")
    irq = (project / "Core/Src/stm32f4xx_it.c").read_text()
    if not re.search(r"void\s+USART2_IRQHandler\s*\(void\)\s*\{[^}]*"
                     r"HAL_UART_IRQHandler\s*\(\s*&huart2\s*\)", irq, re.S):
        errors.append("USART2_IRQHandler must call HAL_UART_IRQHandler(&huart2).")
    if errors:
        raise SystemExit("USART2 receive interrupt setup is incomplete:\n" + "\n".join(errors))


def main():
    project = Path(__file__).resolve().parent
    verify_uart_setup(project)
    compilers = sorted(Path(r"C:\ST").glob("STM32CubeIDE*/**/arm-none-eabi-gcc.exe"))
    if not compilers:
        raise SystemExit("ARM GCC not found under C:\\ST. Use STM32CubeIDE to build.")
    compiler = compilers[-1]
    output = project / "Build_serial_test"
    output.mkdir(exist_ok=True)
    includes = [project / path for path in (
        "Core/Inc", "Drivers/STM32F4xx_HAL_Driver/Inc",
        "Drivers/STM32F4xx_HAL_Driver/Inc/Legacy",
        "Drivers/CMSIS/Device/ST/STM32F4xx/Include", "Drivers/CMSIS/Include")]
    flags = ["-mcpu=cortex-m4", "-mthumb", "-mfpu=fpv4-sp-d16",
             "-mfloat-abi=hard", "-g3", "-O0", "-ffunction-sections", "-fdata-sections"]
    sources = sorted((project / "Core/Src").glob("*.c"))
    sources += sorted((project / "Drivers/STM32F4xx_HAL_Driver/Src").glob("*.c"))
    sources += sorted((project / "Core/Startup").glob("*.s"))
    objects = []
    for source in sources:
        obj = output / ("_".join(source.relative_to(project).parts) + ".o")
        args = [str(compiler), *flags, "-c", str(source), "-o", str(obj)]
        if source.suffix == ".c":
            args += ["-std=gnu11", "-Wall", "-DUSE_HAL_DRIVER", "-DSTM32F446xx"]
            if source.is_relative_to(project / "Core"):
                args += ["-Wextra", "-Werror"]
            args += ["-I" + str(include) for include in includes]
        subprocess.run(args, check=True, cwd=project)
        objects.append(obj)
    elf = output / "servo_sweep_test.elf"
    subprocess.run([str(compiler), *flags, *map(str, objects),
                    "-T" + str(project / "STM32F446RETX_FLASH.ld"),
                    "--specs=nosys.specs", "--specs=nano.specs", "-static",
                    "-Wl,--gc-sections", "-Wl,-Map=" + str(output / "servo_sweep_test.map"),
                    "-Wl,--start-group", "-lc", "-lm", "-Wl,--end-group",
                    "-o", str(elf)], check=True, cwd=project)
    symbols = subprocess.check_output(
        [str(compiler.with_name("arm-none-eabi-nm.exe")), "-g", str(elf)], text=True)
    if not re.search(r"\bT USART2_IRQHandler$", symbols, re.M):
        raise SystemExit("Firmware lacks a strong USART2 interrupt handler.")
    print("USART2 interrupt configuration and linked handler: PASS")
    for format_name, suffix in [("ihex", ".hex"), ("binary", ".bin")]:
        subprocess.run([str(compiler.with_name("arm-none-eabi-objcopy.exe")),
                        "-O", format_name, str(elf), str(elf.with_suffix(suffix))], check=True)
    subprocess.run([str(compiler.with_name("arm-none-eabi-size.exe")), str(elf)], check=True)
    print("Built:", elf)
    print("No board connected or flashed by this script.")


if __name__ == "__main__":
    try:
        main()
    except subprocess.CalledProcessError as error:
        sys.exit(error.returncode)
