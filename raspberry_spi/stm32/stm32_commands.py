"""Camada de aplicação com comandos STM32 expostos no CLI."""

import argparse
import sys
from pathlib import Path
from typing import Any, Dict, List

MODULE_DIR = Path(__file__).resolve().parent

if __package__:
    from .stm32_protocol import (
        REQ_LED_CTRL,
        REQ_MOVE_END,
        REQ_MOVE_HOME,
        REQ_MOVE_PROBE_LEVEL,
        REQ_MOVE_QUEUE_ADD,
        REQ_MOVE_QUEUE_STATUS,
        REQ_START_MOVE,
        REQ_TEST_HELLO,
        RESP_MOVE_END,
        bits_str,
    )
    from .stm32_requests import STM32RequestBuilder
    from .stm32_responses import STM32ResponseDecoder
else:
    if str(MODULE_DIR) not in sys.path:
        sys.path.insert(0, str(MODULE_DIR))
    from stm32_protocol import (  # type: ignore
        REQ_LED_CTRL,
        REQ_MOVE_END,
        REQ_MOVE_HOME,
        REQ_MOVE_PROBE_LEVEL,
        REQ_MOVE_QUEUE_ADD,
        REQ_MOVE_QUEUE_STATUS,
        REQ_START_MOVE,
        REQ_TEST_HELLO,
        RESP_MOVE_END,
        bits_str,
    )
    from stm32_requests import STM32RequestBuilder  # type: ignore
    from stm32_responses import STM32ResponseDecoder  # type: ignore


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


class STM32CommandExecutor:
    def __init__(self, client: Any) -> None:
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
                "Timeout ao aguardar resposta SPI (stm32_client.exchange).",
                f"  comando: {cmd_name}",
                f"  request_type: 0x{request_type:02X}",
                f"  request_payload ({len(request)} bytes): {request_hex}",
                f"  detalhe original: {exc}",
            ]
            raise TimeoutError("\n".join(details)) from exc
        print("Frame RX bits:", bits_str(frame))
        spec = STM32ResponseDecoder.SPECS[request_type]
        try:
            decoded = spec.decoder(frame)
            print(decoded)
            return decoded
        except Exception as exc:
            print("Decoder error:", exc)
            print("Frame RX bits (again):", bits_str(frame))
            raise

    def led_control(self, args: argparse.Namespace) -> None:
        request = STM32RequestBuilder.led_control(
            args.frame_id,
            args.mask,
            args.led1_mode,
            args.led1_freq,
        )
        self._execute_request(REQ_LED_CTRL, request, args)

    def queue_add(self, args: argparse.Namespace) -> None:
        request = STM32RequestBuilder.move_queue_add(
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

    # queue_status removido na versão sucinta

    def start_move(self, args: argparse.Namespace) -> None:
        request = STM32RequestBuilder.start_move(args.frame_id)
        ack = self._execute_request(REQ_START_MOVE, request, args)
        # Opcional: aguarda MOVE_END após ack
        try:
            wait_end = bool(getattr(args, "wait_end", False))
        except Exception:
            wait_end = False
        if wait_end and isinstance(ack, dict) and int(ack.get("status", 0)) == 0:
            # Poll passivo por MOVE_END
            timeout_s = float(getattr(args, "end_timeout", 60.0) or 60.0)
            settle = float(getattr(args, "settle_delay", 0.002) or 0.002)
            tries = max(1, int(timeout_s / max(0.001, settle)))
            try:
                raw = self.client.poll_for(RESP_MOVE_END, expected_len=5, tries=tries, settle_delay_s=settle)
                spec = STM32ResponseDecoder.SPECS[REQ_MOVE_END]
                decoded = spec.decoder(raw)
                print(decoded)
            except Exception as exc:
                print("Aviso: MOVE_END não recebido:", exc)

    def end_move(self, args: argparse.Namespace) -> None:
        request = STM32RequestBuilder.move_end(args.frame_id)
        self._execute_request(REQ_MOVE_END, request, args)

    # Comandos boot_* e hello removidos na versão sucinta


    def safe_off(self, args: argparse.Namespace) -> None:
        """Parar movimento (MOVE_END) e colocar TMC5160 em modo segurança.

        - Envia MOVE_END ao STM32 com o frame-id informado.
        - Ajusta os drivers TMC5160: IHOLD/IRUN=0, TOFF=0 e FREEWHEEL (default=3).
        - Por padrão, afeta todos os /dev/spidevX.Y quando --tmc-bus/dev não são passados.
        """

        # 1) Solicitar parada ao controlador
        try:
            request = STM32RequestBuilder.move_end(args.frame_id)
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

    def set_microsteps_axes(self, args: argparse.Namespace) -> None:
        # Envia REQ_SET_MICROSTEPS_AX com ms por eixo
        try:
            from .stm32_protocol import REQ_SET_MICROSTEPS_AX  # type: ignore
        except Exception:
            from stm32_protocol import REQ_SET_MICROSTEPS_AX  # type: ignore
        request = STM32RequestBuilder.set_microsteps_axes(int(args.frame_id) & 0xFF, int(args.x), int(args.y), int(args.z))
        # Expect no decoder; just print bits
        frame = self.client.exchange(REQ_SET_MICROSTEPS_AX, request, tries=int(getattr(args, "tries", 2) or 2), settle_delay_s=float(getattr(args, "settle_delay", 0.002) or 0.002))
        print("Frame RX bits:", bits_str(frame))

__all__ = ["STM32CommandExecutor", "print_boot_frame_info"]
