#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Gera um gráfico didático das curvas de saída do transistor BC548,
mostrando:
- família de curvas para diferentes correntes de base I_B;
- regiões de corte, ativa e saturação;
- linha de carga de um relé de 5 V com bobina de 70 ohms;
- ponto de operação estimado para R_B = 10 kOhm.

Observação:
As faixas e limites principais foram inspirados nos valores do datasheet
do BC548, mas as curvas completas I_C x V_CE para vários I_B foram
construídas de forma didática/aproximada para fins educacionais.
"""

from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt


def gerar_grafico(output_path="img/grafico_bc548_datasheet_curvas_ib_rb10k.png"):
    # ------------------------------------------------------------------
    # Parâmetros do circuito / transistor
    # ------------------------------------------------------------------
    VCC = 5.0          # tensão da fonte do relé (V)
    RL = 70.0          # resistência da bobina do relé (ohm)
    RB = 10_000.0      # resistor de base (ohm)

    # Valores típicos usados como referência didática
    VBE = 0.70         # V_BE típico (V)
    VCEsat_typ = 0.20  # V_CE(sat) típico (V)
    VCEsat_max = 0.60  # V_CE(sat) máximo (V)
    IC_MAX = 100.0     # corrente contínua máxima de coletor (mA)

    # Ganho típico apenas para modelagem didática das curvas
    beta_typical = 200

    # Família de correntes de base para ilustrar corte / ativa / saturação
    IB_values_mA = [0.00, 0.05, 0.10, 0.20, 0.50, 1.00, 2.00, 5.00]

    # Eixo V_CE
    VCE = np.linspace(0, VCC, 600)

    fig, ax = plt.subplots(figsize=(10, 6.5))

    # ------------------------------------------------------------------
    # Curvas de saída aproximadas do transistor
    # ------------------------------------------------------------------
    for IB_mA in IB_values_mA:
        if IB_mA == 0:
            IC_curve = np.zeros_like(VCE)
            label = r"$I_B = 0$ mA"
        else:
            # Corrente de coletor na região ativa (aproximada)
            IC_plateau = min(beta_typical * IB_mA, IC_MAX)

            # Ajuste do "joelho" da curva para suavizar a transição
            knee = 0.12 + 0.02 * np.log10(1 + 20 * IB_mA)

            IC_curve = IC_plateau * (1 - np.exp(-VCE / knee))
            IC_curve = np.minimum(IC_curve, IC_MAX)

            label = (
                rf"$I_B = {IB_mA:.2f}$ mA"
                if IB_mA < 1
                else rf"$I_B = {IB_mA:.1f}$ mA"
            )

        ax.plot(VCE, IC_curve, label=label)

    # ------------------------------------------------------------------
    # Linha de carga do relé
    # ------------------------------------------------------------------
    IC_load = (VCC - VCE) / RL * 1000  # mA
    IC_load = np.clip(IC_load, 0, None)

    ax.plot(
        VCE,
        IC_load,
        linestyle="--",
        linewidth=2.2,
        label="Linha de carga do relé (5 V, 70 Ω)"
    )

    # ------------------------------------------------------------------
    # Destaque da região de saturação
    # ------------------------------------------------------------------
    ax.axvspan(0, VCEsat_typ, alpha=0.15)
    ax.axvspan(VCEsat_typ, VCEsat_max, alpha=0.07)

    # ------------------------------------------------------------------
    # Ponto estimado para R_B = 10k
    # ------------------------------------------------------------------
    IB_rb10k_mA = (VCC - VBE) / RB * 1000

    IC_plateau_rb10k = min(beta_typical * IB_rb10k_mA, IC_MAX)
    IC_curve_rb10k = IC_plateau_rb10k * (1 - np.exp(-VCE / 0.13))

    diff = np.abs(IC_curve_rb10k - IC_load)
    idx = np.argmin(diff)

    VCE_q = float(VCE[idx])
    IC_q = float(IC_load[idx])

    ax.plot(
        [VCE_q],
        [IC_q],
        marker="o",
        markersize=8,
        label=r"Ponto estimado para $R_B = 10\,k\Omega$"
    )

    ax.annotate(
        "R_B = 10 kΩ\n"
        f"I_B ≈ {IB_rb10k_mA:.2f} mA\n"
        f"V_CE ≈ {VCE_q:.2f} V\n"
        f"I_C ≈ {IC_q:.1f} mA",
        xy=(VCE_q, IC_q),
        xytext=(1.25, 64),
        arrowprops=dict(arrowstyle="->"),
        fontsize=10
    )

    # ------------------------------------------------------------------
    # Guias adicionais
    # ------------------------------------------------------------------
    ax.axhline(
        IC_MAX,
        linestyle=":",
        linewidth=1.5,
        label="I_C máx. contínua do datasheet = 100 mA"
    )

    # Textos das regiões
    ax.text(0.05, 92, "Saturação", fontsize=11)
    ax.text(0.24, 84, "faixa típica", fontsize=9)
    ax.text(0.32, 74, "até 0,6 V", fontsize=9)
    ax.text(2.7, 83, "Região ativa", fontsize=12)
    ax.text(4.05, 5.5, "Corte", fontsize=12)

    # Título e eixos
    ax.set_title(
        "BC548: curvas de saída didáticas com região de saturação\n"
        "(baseadas nos valores do datasheet e no driver de relé 5 V)"
    )
    ax.set_xlabel(r"Tensão coletor-emissor $V_{CE}$ (V)")
    ax.set_ylabel(r"Corrente de coletor $I_C$ (mA)")
    ax.set_xlim(0, 5)
    ax.set_ylim(0, 105)
    ax.grid(True, alpha=0.35)
    ax.legend(loc="center right", fontsize=8)

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    plt.tight_layout()
    plt.savefig(output_path, dpi=220, bbox_inches="tight")
    plt.close(fig)

    print(f"Gráfico salvo em: {output_path.resolve()}")


if __name__ == "__main__":
    gerar_grafico()
