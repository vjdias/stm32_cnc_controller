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
"""

from __future__ import annotations

import os
import shlex
import subprocess
import sys
import unittest
from pathlib import Path
from typing import Iterable, List


MODULE_DIR = Path(__file__).resolve().parent


def _format_command(cmd: Iterable[str]) -> str:
    """Converte o comando em uma string amigável para logs/assertivas."""

    return " ".join(shlex.quote(token) for token in cmd)


class TestSTM32SpiClient(unittest.TestCase):
    """Testa os serviços ``hello`` e ``led-control`` via ``cnc_spi_client.py``.

    Cada método reflete exatamente os passos descritos na rotina manual de
    validação:

    * ``test_hello_single_try`` – valida o enlace básico com ``--tries 1``.
    * ``test_hello_with_polling`` – força múltiplos ciclos de polling com
      ``--tries 5`` e atraso configurado.
    * ``test_led_control_static`` – aciona o LED no modo contínuo.
    * ``test_led_control_blink`` – programa o LED para piscar em 0,5 Hz.

    Os testes verificam que o comando finaliza com ``returncode`` igual a zero
    e que a saída não contém mensagens de ``BUSY`` ou rastros de exceção,
    indicando que o STM32 respondeu conforme esperado.
    """

    requires_hw = os.environ.get("STM32_SPI_AVAILABLE") == "1"

    @classmethod
    def setUpClass(cls) -> None:  # pragma: no cover - skip logic
        if not cls.requires_hw:
            raise unittest.SkipTest(
                "STM32_SPI_AVAILABLE!=1 – testes de hardware ignorados"
            )

    def _run_client(self, *args: str) -> subprocess.CompletedProcess[str]:
        """Executa o script ``cnc_spi_client.py`` com argumentos fornecidos.

        O método captura ``stdout``/``stderr`` e impede que exceções não
        tratadas cheguem até o runner de testes.  Caso o comando retorne um
        código diferente de zero ou registre mensagens de erro conhecidas, a
        asserção falha com detalhes do comando e da saída capturada.
        """

        command: List[str] = [
            sys.executable,
            "cnc_spi_client.py",
            *args,
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

        return result

    def test_hello_single_try(self) -> None:
        """Validação básica do enlace executando ``hello`` com ``--tries 1``."""

        self._run_client("hello", "--tries", "1")

    def test_hello_with_polling(self) -> None:
        """Repete o polling da resposta do ``hello`` com múltiplas tentativas."""

        self._run_client("hello", "--tries", "5", "--settle-delay", "0.002")

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
            "--tries",
            "5",
            "--settle-delay",
            "0.002",
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
            "--tries",
            "5",
            "--settle-delay",
            "0.002",
        )


if __name__ == "__main__":  # pragma: no cover - execução manual
    unittest.main()
