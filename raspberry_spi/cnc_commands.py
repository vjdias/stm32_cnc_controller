"""Camada de aplicação com comandos disponíveis via CLI."""

import argparse
import sys
from pathlib import Path
from typing import Any, Dict, List

MODULE_DIR = Path(__file__).resolve().parent

if __package__:
    from .cnc_client import CNCClient
    from .cnc_protocol import (
        REQ_LED_CTRL,
        REQ_MOVE_END,
        REQ_MOVE_HOME,
        REQ_MOVE_PROBE_LEVEL,
        REQ_MOVE_QUEUE_ADD,
        REQ_MOVE_QUEUE_STATUS,
        REQ_START_MOVE,
        REQ_TEST_HELLO,
        bits_str,
    )
    from .cnc_requests import CNCRequestBuilder
    from .cnc_responses import CNCResponseDecoder
else:
    if str(MODULE_DIR) not in sys.path:
        sys.path.insert(0, str(MODULE_DIR))
    from cnc_client import CNCClient  # type: ignore
    from cnc_protocol import (  # type: ignore
        REQ_LED_CTRL,
        REQ_MOVE_END,
        REQ_MOVE_HOME,
        REQ_MOVE_PROBE_LEVEL,
        REQ_MOVE_QUEUE_ADD,
        REQ_MOVE_QUEUE_STATUS,
        REQ_START_MOVE,
        REQ_TEST_HELLO,
        bits_str,
    )
    from cnc_requests import CNCRequestBuilder  # type: ignore
    from cnc_responses import CNCResponseDecoder  # type: ignore


def print_boot_frame_info(frame: List[int], stats: Dict[str, Any]) -> None:
    print(" ".join(f"{b:02X}" for b in frame))
    print("Frame RX bits:", bits_str(frame))
    summary_keys = ("bytesBeforeHeader", "bytesUntilTail", "readsUsed", "chunkLen")
    print({k: stats[k] for k in summary_keys})
    handshake_bytes = stats.get("handshakeBytes", [])
    if handshake_bytes:
        print(
            "handshake:",
            " ".join(f"{b:02X}" for b in handshake_bytes),
            f"({bits_str(handshake_bytes)})",
        )
    else:
        print("handshake: []")
    chunks = stats.get("chunks", [])
    print(f"chunks recebidos: {len(chunks)}")
    for idx, chunk in enumerate(chunks):
        print(f"chunk {idx:02d}:", " ".join(f"{b:02X}" for b in chunk))
        print(f"chunk {idx:02d} bits:", bits_str(chunk))
    if isinstance(frame, list) and len(frame) >= 2:
        cmd_byte = frame[1]
        cmd_chr = chr(cmd_byte) if 32 <= cmd_byte <= 126 else "?"
        print(f"comando encontrado: 0x{cmd_byte:02X} ('{cmd_chr}')")


class CNCCommandExecutor:
    def __init__(self, client: CNCClient) -> None:
        self.client = client

    def _execute_request(
        self, request_type: int, request: List[int], args: argparse.Namespace
    ) -> Dict[str, Any]:
        kwargs: Dict[str, Any] = {}
        tries = getattr(args, "tries", None)
        if tries is not None:
            kwargs["tries"] = tries
        settle_delay = getattr(args, "settle_delay", None)
        if settle_delay is not None:
            kwargs["settle_delay_s"] = settle_delay
        poll_byte = getattr(args, "poll_byte", None)
        disable_poll = getattr(args, "disable_poll", False)
        if disable_poll:
            kwargs["poll_byte"] = None
        elif poll_byte is not None:
            kwargs["poll_byte"] = poll_byte

        try:
            frame = self.client.exchange(request_type, request, **kwargs)
        except TimeoutError as exc:
            cmd_name = getattr(args, "command", f"0x{request_type:02X}")
            request_hex = " ".join(f"{b & 0xFF:02X}" for b in request)
            details = [
                "Timeout ao aguardar resposta SPI (cnc_client.exchange).",
                f"  comando: {cmd_name}",
                f"  request_type: 0x{request_type:02X}",
                f"  request_payload ({len(request)} bytes): {request_hex}",
                f"  detalhe original: {exc}",
            ]
            raise TimeoutError("\n".join(details)) from exc
        print("Frame RX bits:", bits_str(frame))
        spec = CNCResponseDecoder.SPECS[request_type]
        try:
            decoded = spec.decoder(frame)
            print(decoded)
            return decoded
        except Exception as exc:
            print("Decoder error:", exc)
            print("Frame RX bits (again):", bits_str(frame))
            raise

    def led_control(self, args: argparse.Namespace) -> None:
        request = CNCRequestBuilder.led_control(
            args.frame_id,
            args.mask,
            args.led1_mode,
            args.led1_freq,
        )
        self._execute_request(REQ_LED_CTRL, request, args)

    def queue_add(self, args: argparse.Namespace) -> None:
        request = CNCRequestBuilder.move_queue_add(
            args.frame_id,
            args.dir,
            args.vx,
            args.sx,
            args.vy,
            args.sy,
            args.vz,
            args.sz,
            args.kp_x,
            args.ki_x,
            args.kd_x,
            args.kp_y,
            args.ki_y,
            args.kd_y,
            args.kp_z,
            args.ki_z,
            args.kd_z,
        )
        self._execute_request(REQ_MOVE_QUEUE_ADD, request, args)

    def queue_status(self, args: argparse.Namespace) -> None:
        request = CNCRequestBuilder.queue_status(args.frame_id)
        self._execute_request(REQ_MOVE_QUEUE_STATUS, request, args)

    def start_move(self, args: argparse.Namespace) -> None:
        request = CNCRequestBuilder.start_move(args.frame_id)
        self._execute_request(REQ_START_MOVE, request, args)

    def end_move(self, args: argparse.Namespace) -> None:
        request = CNCRequestBuilder.move_end(args.frame_id)
        self._execute_request(REQ_MOVE_END, request, args)

    def home(self, args: argparse.Namespace) -> None:
        request = CNCRequestBuilder.move_home(args.frame_id, args.axes, args.dirs, args.vhome)
        self._execute_request(REQ_MOVE_HOME, request, args)

    def probe_level(self, args: argparse.Namespace) -> None:
        request = CNCRequestBuilder.probe_level(args.frame_id, args.axes, args.vprobe)
        self._execute_request(REQ_MOVE_PROBE_LEVEL, request, args)

    def hello(self, args: argparse.Namespace) -> None:
        request = CNCRequestBuilder.hello()
        decoded = self._execute_request(REQ_TEST_HELLO, request, args)
        if decoded:
            print(decoded.get("payload"))

    def boot_hello(self, args: argparse.Namespace) -> None:
        try:
            frame, stats = self.client.read_boot_hello_info(
                tries=args.tries,
                settle_delay_s=args.settle_delay,
                chunk_len=args.chunk_len,
            )
        except TimeoutError:
            print(
                "Frame 'hello' não encontrado. Ative APP_ENABLE_BOOT_TEST_RESPONSES "
                "no firmware ou utilize o comando 'hello'."
            )
            return
        print_boot_frame_info(frame, stats)

    def boot_led(self, args: argparse.Namespace) -> None:
        try:
            frame, stats = self.client.read_boot_led_info(
                tries=args.tries,
                settle_delay_s=args.settle_delay,
                chunk_len=args.chunk_len,
            )
        except TimeoutError:
            print(
                "Frame 'led' não encontrado. Certifique-se de que o firmware "
                "publica esse frame de boot antes de usar este comando."
            )
            return
        print_boot_frame_info(frame, stats)


    def safe_off(self, args: argparse.Namespace) -> None:
        """Parar movimento (MOVE_END) e colocar TMC5160 em modo segurança.

        - Envia MOVE_END ao STM32 com o frame-id informado.
        - Ajusta os drivers TMC5160: IHOLD/IRUN=0, TOFF=0 e FREEWHEEL (default=3).
        - Por padrão, afeta todos os /dev/spidevX.Y quando --tmc-bus/dev não são passados.
        """

        # 1) Solicitar parada ao controlador
        try:
            request = CNCRequestBuilder.move_end(args.frame_id)
            self._execute_request(REQ_MOVE_END, request, args)
        except Exception as exc:
            print("Aviso: falha ao enviar MOVE_END ao STM32:", exc)

        # 2) Modo segurança nos TMC5160 (opcional)
        if getattr(args, "skip_tmc", False):
            print("Skip TMC: apenas MOVE_END enviado.")
            return

        # Import tardio para evitar custo quando não usado
        try:
            if __package__:
                from .tmc5160 import (
                    TMC5160Configurator,
                    TMC5160RegisterPreset,
                    REG_GSTAT,
                    REG_IHOLD_IRUN,
                    REG_CHOPCONF,
                    REG_PWMCONF,
                )
            else:  # execução direta
                from tmc5160 import (  # type: ignore
                    TMC5160Configurator,
                    TMC5160RegisterPreset,
                    REG_GSTAT,
                    REG_IHOLD_IRUN,
                    REG_CHOPCONF,
                    REG_PWMCONF,
                )
        except Exception as exc:  # pragma: no cover
            print("TMC5160 não disponível neste ambiente:", exc)
            return

        from pathlib import Path as _Path

        def _parse_spidev(path) -> tuple[int, int] | None:
            name = str(path).rsplit("/", 1)[-1]
            if not name.startswith("spidev"):
                return None
            try:
                rest = name[len("spidev") :]
                bus_str, dev_str = rest.split(".", 1)
                return int(bus_str), int(dev_str)
            except Exception:
                return None

        tmc_bus = getattr(args, "tmc_bus", None)
        tmc_dev = getattr(args, "tmc_dev", None)
        targets: list[tuple[int, int]] = []
        if tmc_bus is None and tmc_dev is None:
            for p in sorted(_Path("/dev").glob("spidev*")):
                parsed = _parse_spidev(p)
                if parsed is not None:
                    targets.append(parsed)
        else:
            if tmc_bus is None or tmc_dev is None:
                print("Informe --tmc-bus e --tmc-dev juntos, ou omita ambos para agir em todos.")
                return
            targets = [(int(tmc_bus), int(tmc_dev))]

        ihold_irun_safe = 0x00000000
        chopconf_base = 0x14010053
        chopconf_safe = chopconf_base & ~0x0F  # TOFF=0
        pwmconf_base = 0xC10D0024
        freewheel = int(getattr(args, "tmc_freewheel", 3)) & 0x3
        pwmconf_safe = (pwmconf_base & ~(0x3 << 20)) | (freewheel << 20)

        for bus, dev in sorted(set(targets)):
            print(f"TMC SAFE-OFF em /dev/spidev{bus}.{dev} (FREEWHEEL={freewheel})")
            configurator = TMC5160Configurator(
                bus=bus,
                device=dev,
                speed_hz=getattr(args, "tmc_speed", 4_000_000),
                register_preset=TMC5160RegisterPreset(writes=tuple()),
            )
            try:
                with configurator as driver:  # type: ignore
                    writes = [
                        (REG_GSTAT, 0x00000007),
                        (REG_IHOLD_IRUN, ihold_irun_safe),
                        (REG_CHOPCONF, chopconf_safe),
                        (REG_PWMCONF, pwmconf_safe),
                    ]
                    responses = driver.apply_registers(writes)
                    for r in responses:
                        print(f" - 0x{r.address:02X} <= 0x{r.value:08X}")
                        r.raise_on_faults()
            except Exception as exc:
                print(f"Aviso: falha ao aplicar SAFE-OFF no TMC /dev/spidev{bus}.{dev}: {exc}")

__all__ = ["CNCCommandExecutor", "print_boot_frame_info"]
