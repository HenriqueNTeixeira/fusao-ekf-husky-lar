#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analisa_resultados.py
Le os CSVs das tres configuracoes e gera os graficos comparativos:
  - trajetorias sobrepostas
  - erro de posicao x tempo
  - barras de RMSE / erro final / erro de orientacao
  - CDF do erro de posicao
Imprime e salva uma tabela-resumo.

SEM dependencia de pandas (so numpy + csv + matplotlib), para rodar
direto no container do ROS sem conflito de versao.

Uso:
  python3 analise/analisa_resultados.py [pasta_base]
Por padrao procura ~/resultados_fusao/<config>/erros.csv
"""
import os
import sys
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

CONFIGS = ["odom", "odom_imu", "odom_imu_gps"]
ROTULOS = {
    "odom": "Odometria",
    "odom_imu": "Odom + IMU",
    "odom_imu_gps": "Odom + IMU + GPS",
}
CORES = {"odom": "#d62728", "odom_imu": "#1f77b4", "odom_imu_gps": "#2ca02c"}


def ler_csv(path):
    """Le erros.csv e devolve dict de colunas (listas de float)."""
    cols = {"t": [], "err_pos_m": [], "err_yaw_rad": [],
            "gt_x": [], "gt_y": [], "est_x": [], "est_y": []}
    with open(path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            for k in cols:
                cols[k].append(float(row[k]))
    for k in cols:
        cols[k] = np.array(cols[k])
    return cols


def carregar(base):
    dados = {}
    for c in CONFIGS:
        path = os.path.join(base, c, "erros.csv")
        if os.path.isfile(path):
            dados[c] = ler_csv(path)
        else:
            print("[aviso] nao encontrei %s (config '%s' ignorada)" % (path, c))
    return dados


def resumo(dados):
    linhas = []
    for c in CONFIGS:
        if c not in dados:
            continue
        err = dados[c]["err_pos_m"]
        eyaw = dados[c]["err_yaw_rad"]
        linhas.append({
            "config": ROTULOS[c],
            "amostras": len(err),
            "RMSE_pos_m": float(np.sqrt(np.mean(err ** 2))),
            "erro_final_m": float(err[-1]),
            "erro_max_m": float(np.max(err)),
            "p95_m": float(np.percentile(err, 95)),
            "RMSE_yaw_deg": float(np.degrees(np.sqrt(np.mean(eyaw ** 2)))),
        })
    return linhas


def imprime_tabela(linhas):
    cols = ["config", "amostras", "RMSE_pos_m", "erro_final_m",
            "erro_max_m", "p95_m", "RMSE_yaw_deg"]
    larg = {c: max(len(c), 16) for c in cols}
    cab = "  ".join(c.ljust(larg[c]) for c in cols)
    print("\n===== RESUMO COMPARATIVO =====")
    print(cab)
    for ln in linhas:
        vals = []
        for c in cols:
            v = ln[c]
            if c == "config":
                s = str(v)
            elif c == "amostras":
                s = "%d" % v
            else:
                s = "%.4f" % v
            vals.append(s.ljust(larg[c]))
        print("  ".join(vals))


def salva_resumo_csv(linhas, out):
    cols = ["config", "amostras", "RMSE_pos_m", "erro_final_m",
            "erro_max_m", "p95_m", "RMSE_yaw_deg"]
    with open(os.path.join(out, "resumo_comparativo.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for ln in linhas:
            w.writerow(ln)


def plot_trajetorias(dados, out):
    plt.figure(figsize=(7, 6))
    primeiro = dados[next(iter(dados))]
    plt.plot(primeiro["gt_x"], primeiro["gt_y"], "k-", lw=2.5,
             label="Ground truth")
    for c in CONFIGS:
        if c in dados:
            plt.plot(dados[c]["est_x"], dados[c]["est_y"], "--", lw=1.5,
                     color=CORES[c], label=ROTULOS[c])
    plt.axis("equal")
    plt.grid(True, ls=":")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title("Trajetorias estimadas x ground truth")
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(out, "comp_trajetorias.png"), dpi=140)
    plt.close()


def plot_erro_tempo(dados, out):
    plt.figure(figsize=(9, 4.5))
    for c in CONFIGS:
        if c in dados:
            plt.plot(dados[c]["t"], dados[c]["err_pos_m"], lw=1.3,
                     color=CORES[c], label=ROTULOS[c])
    plt.grid(True, ls=":")
    plt.xlabel("tempo (s)")
    plt.ylabel("erro de posicao (m)")
    plt.title("Erro de posicao x tempo")
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(out, "comp_erro_tempo.png"), dpi=140)
    plt.close()


def plot_barras(linhas, out):
    nomes = [ln["config"] for ln in linhas]
    cores = [CORES[c] for c in CONFIGS if ROTULOS[c] in nomes]
    fig, axs = plt.subplots(1, 3, figsize=(12, 4))
    metr = [("RMSE_pos_m", "RMSE de posicao (m)"),
            ("erro_final_m", "Erro final (m)"),
            ("RMSE_yaw_deg", "RMSE de orientacao (deg)")]
    for ax, (col, titulo) in zip(axs, metr):
        ax.bar(nomes, [ln[col] for ln in linhas], color=cores)
        ax.set_title(titulo)
        ax.tick_params(axis="x", rotation=20)
        ax.grid(True, axis="y", ls=":")
    plt.tight_layout()
    plt.savefig(os.path.join(out, "comp_barras.png"), dpi=140)
    plt.close()


def plot_cdf(dados, out):
    plt.figure(figsize=(7, 5))
    for c in CONFIGS:
        if c in dados:
            err = np.sort(dados[c]["err_pos_m"])
            cdf = np.arange(1, len(err) + 1) / len(err)
            plt.plot(err, cdf, lw=1.6, color=CORES[c], label=ROTULOS[c])
    plt.grid(True, ls=":")
    plt.xlabel("erro de posicao (m)")
    plt.ylabel("fracao acumulada")
    plt.title("CDF do erro de posicao")
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(out, "comp_cdf.png"), dpi=140)
    plt.close()


def main():
    base = sys.argv[1] if len(sys.argv) > 1 else \
        os.path.join(os.path.expanduser("~"), "resultados_fusao")
    out = os.path.join(base, "comparacao")
    if not os.path.isdir(out):
        os.makedirs(out)

    dados = carregar(base)
    if not dados:
        print("Nenhum CSV encontrado em %s" % base)
        sys.exit(1)

    linhas = resumo(dados)
    imprime_tabela(linhas)
    salva_resumo_csv(linhas, out)

    plot_trajetorias(dados, out)
    plot_erro_tempo(dados, out)
    plot_barras(linhas, out)
    plot_cdf(dados, out)
    print("\nFiguras e resumo salvos em: %s" % out)


if __name__ == "__main__":
    main()
