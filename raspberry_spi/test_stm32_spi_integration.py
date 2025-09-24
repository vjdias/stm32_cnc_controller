
"""Testes integrados para validar o fluxo STM32 ↔ Raspberry Pi via CLI.

Este módulo reúne cenários de teste que exercitam diretamente o script
``cnc_spi_client.py`` utilizando os parâmetros descritos na rotina manual de
validação.  O objetivo é confirmar que os serviços expostos pelo cliente
executam sem disparar exceções e que o fluxo de handshake/resposta se comporta
como esperado quando o hardware real (STM32 conectado ao Raspberry Pi) está
disponível.

Para evitar falhas em ambientes sem acesso ao barramento SPI real, os testes
são condicionados à variável de ambiente ``STM32_SPI_AVAILABLE``.  Quando ela
não está definida com valor ``"1"``, todo o conjunto é automaticamente
ignorado.

Também é possível personalizar o comportamento de polling usando as variáveis
``STM32_SPI_TEST_TRIES`` (tentativas), ``STM32_SPI_TEST_SETTLE_DELAY``
(intervalo entre tentativas em segundos) e ``STM32_SPI_TEST_WAIT_SECONDS``
(pausa pós-comando).  Quando não configuradas, os valores padrão adotam cinco
tentativas, ``0.002`` segundo de atraso e nenhuma espera adicional.
"""

from __future__ import annotations

import os
import shlex
import subprocess
import sys
import time
import unittest
from pathlib import Path
from typing import Iterable, List, Optional


MODULE_DIR = Path(__file__).resolve().parent


def _env_int(name: str, fallback: Optional[int]) -> Optional[int]:
    """Retorna um inteiro opcional a partir da variável de ambiente."""

    value = os.environ.get(name)
    if value is None or not value.strip():
        return fallback
    return int(value)


def _env_float(name: str, fallback: Optional[float]) -> Optional[float]:
    """Retorna um float opcional a partir da variável de ambiente."""

    value = os.environ.get(name)
    if value is None or not value.strip():
        return fallback
    return float(value)


def _format_command(cmd: Iterable[str]) -> str:
    """Converte o comando em uma string amigável para logs/assertivas."""

    return " ".join(shlex.quote(token) for token in cmd)


class TestSTM32SpiClient(unittest.TestCase):
    """Testa os serviços ``hello`` e ``led-control`` via ``cnc_spi_client.py``.

    Cada método reflete exatamente os passos descritos na rotina manual de
    validação:

    * ``test_hello_single_try`` – valida o enlace básico com ``--tries 1``.
    * ``test_hello_with_polling`` – força múltiplos ciclos de polling com
    # ``--tries 5`` e atraso configurado.
    * ``test_led_control_static`` – aciona o LED no modo contínuo.
    * ``test_led_control_blink`` – programa o LED para piscar em 0,5 Hz.

    Os testes verificam que o comando finaliza com ``returncode`` igual a zero
    e que a saída não contém mensagens de ``BUSY`` ou rastros de exceção,
    indicando que o STM32 respondeu conforme esperado.
    """

    requires_hw = os.environ.get("STM32_SPI_AVAILABLE") == "1"

    default_tries = _env_int("STM32_SPI_TEST_TRIES", 5)
    default_settle_delay = _env_float("STM32_SPI_TEST_SETTLE_DELAY", 0.002)
    default_wait_seconds = _env_float("STM32_SPI_TEST_WAIT_SECONDS", 0.0)

    @classmethod
    def setUpClass(cls) -> None:  # pragma: no cover - skip logic
        if not cls.requires_hw:
            raise unittest.SkipTest(
                "STM32_SPI_AVAILABLE!=1 – testes de hardware ignorados"
            )

    def _run_client(
        self,
        *args: str,
        tries: Optional[int] = None,
        settle_delay: Optional[float] = None,
        wait_seconds: Optional[float] = None,
    ) -> subprocess.CompletedProcess[str]:
        """Executa o script ``cnc_spi_client.py`` com argumentos fornecidos.

        O método captura ``stdout``/``stderr`` e impede que exceções não
        tratadas cheguem até o runner de testes.  Caso o comando retorne um
        código diferente de zero ou registre mensagens de erro conhecidas, a
        asserção falha com detalhes do comando e da saída capturada.
        """

        tries = self.default_tries if tries is None else tries
        settle_delay = self.default_settle_delay if settle_delay is None else settle_delay
        wait_seconds = self.default_wait_seconds if wait_seconds is None else wait_seconds

        command_args: List[str] = list(args)
        if tries is not None:
            command_args.extend(["--tries", str(tries)])
        if settle_delay is not None:
            command_args.extend(["--settle-delay", str(settle_delay)])

        command: List[str] = [
            sys.executable,
            "cnc_spi_client.py",
            *command_args,
        ]
        result = subprocess.run(
            command,
            check=False,
            cwd=MODULE_DIR,
            capture_output=True,
            text=True,
        )

        output = f"STDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}"
        cmd_repr = _format_command(command)

        self.assertEqual(
            result.returncode,
            0,
            msg=f"Comando falhou ({cmd_repr})\n{output}",
        )
        self.assertNotIn(
            "BUSY",
            output,
            msg=f"Handshake sinalizou BUSY inesperado ({cmd_repr})\n{output}",
        )
        self.assertNotIn(
            "Traceback",
            output,
            msg=f"Exceção não tratada capturada ({cmd_repr})\n{output}",
        )

        if wait_seconds and wait_seconds > 0:
            time.sleep(wait_seconds)

        return result

    def test_hello_single_try(self) -> None:
        """Validação básica do enlace executando ``hello`` com ``--tries 1``."""

        self._run_client("hello", tries=1)

    def test_hello_with_polling(self) -> None:
        """Repete o polling da resposta do ``hello`` com múltiplas tentativas."""

        self._run_client("hello")

    def test_led_control_static(self) -> None:
        """Ativa o LED discreto no modo contínuo para validar o serviço."""

        self._run_client(
            "led-control",
            "--frame-id",
            "1",
            "--mask",
            "0x01",
            "--led1-mode",
            "1",
        )

    def test_led_control_blink(self) -> None:
        """Programa o LED discreto para piscar em 0,5 Hz usando ``led-control``."""

        self._run_client(
            "led-control",
            "--frame-id",
            "2",
            "--mask",
            "0x01",
            "--led1-mode",
            "2",
            "--led1-freq",
            "0.5",
        )


if __name__ == "__main__":  # pragma: no cover - execução manual
    unittest.main()
