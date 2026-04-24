"""
remove_white_background.py

Remove fundo branco/quase branco de imagens e salva como PNG com transparência.

Uso individual:
    python remove_white_background.py imagem.png

Uso individual com saída:
    python remove_white_background.py imagem.png saida_sem_fundo.png

Uso em pasta:
    python remove_white_background.py --batch imagens imagens_sem_fundo

Ajuste de sensibilidade:
    python remove_white_background.py imagem.png --threshold 240

Quanto menor o threshold, mais agressiva é a remoção.
Valores úteis: 230, 235, 240, 245, 250.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


VALID_EXTENSIONS = {".png", ".jpg", ".jpeg", ".webp", ".bmp", ".tif", ".tiff"}


def remove_white_background(
    input_path: str | Path,
    output_path: str | Path | None = None,
    threshold: int = 245,
    smooth_edges: bool = True,
) -> Path:
    """
    Remove o fundo branco ou quase branco de uma imagem.

    A imagem de saída é salva como PNG com canal alfa, ou seja,
    com transparência real.

    Parameters
    ----------
    input_path:
        Caminho da imagem original.

    output_path:
        Caminho da imagem de saída. Se None, cria automaticamente
        um arquivo com sufixo "_sem_fundo.png".

    threshold:
        Valor mínimo para considerar um pixel como fundo branco.
        Quanto menor, mais agressiva será a remoção.

    smooth_edges:
        Se True, suaviza levemente as bordas da transparência.

    Returns
    -------
    Path
        Caminho do arquivo salvo.
    """

    input_file = Path(input_path)

    if output_path is None:
        output_file = input_file.with_name(f"{input_file.stem}_sem_fundo.png")
    else:
        output_file = Path(output_path)

    output_file.parent.mkdir(parents=True, exist_ok=True)

    image = cv2.imread(str(input_file), cv2.IMREAD_COLOR)

    if image is None:
        raise FileNotFoundError(f"Não foi possível abrir a imagem: {input_file}")

    # OpenCV lê em BGR. Convertendo para RGB para facilitar a lógica.
    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Pixels quase brancos viram fundo.
    white_mask = (
        (rgb[:, :, 0] >= threshold)
        & (rgb[:, :, 1] >= threshold)
        & (rgb[:, :, 2] >= threshold)
    ).astype(np.uint8) * 255

    # Pequena limpeza morfológica para reduzir ruídos isolados.
    kernel = np.ones((3, 3), np.uint8)
    white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)

    # Alfa: objeto opaco, fundo transparente.
    alpha = 255 - white_mask

    if smooth_edges:
        alpha = cv2.GaussianBlur(alpha, (3, 3), 0)
        alpha[alpha > 250] = 255
        alpha[alpha < 5] = 0

    b, g, r = cv2.split(image)
    result = cv2.merge([b, g, r, alpha])

    success = cv2.imwrite(str(output_file), result)

    if not success:
        raise RuntimeError(f"Não foi possível salvar a imagem em: {output_file}")

    return output_file


def process_folder(
    input_folder: str | Path,
    output_folder: str | Path,
    threshold: int = 245,
    smooth_edges: bool = True,
) -> list[Path]:
    """
    Processa todas as imagens válidas de uma pasta.

    Parameters
    ----------
    input_folder:
        Pasta contendo as imagens originais.

    output_folder:
        Pasta onde serão salvas as imagens com fundo removido.

    threshold:
        Valor mínimo para detectar fundo branco.

    smooth_edges:
        Se True, suaviza as bordas da transparência.

    Returns
    -------
    list[Path]
        Lista com os arquivos gerados.
    """

    input_dir = Path(input_folder)
    output_dir = Path(output_folder)
    output_dir.mkdir(parents=True, exist_ok=True)

    if not input_dir.exists():
        raise FileNotFoundError(f"Pasta não encontrada: {input_dir}")

    generated_files: list[Path] = []

    for image_path in sorted(input_dir.iterdir()):
        if image_path.suffix.lower() not in VALID_EXTENSIONS:
            continue

        output_path = output_dir / f"{image_path.stem}_sem_fundo.png"

        generated = remove_white_background(
            input_path=image_path,
            output_path=output_path,
            threshold=threshold,
            smooth_edges=smooth_edges,
        )

        generated_files.append(generated)
        print(f"Processado: {image_path.name} -> {generated.name}")

    return generated_files


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Remove fundo branco/quase branco de imagens e salva como PNG transparente."
    )

    parser.add_argument(
        "input",
        help="Imagem de entrada ou pasta de entrada, caso use --batch.",
    )

    parser.add_argument(
        "output",
        nargs="?",
        help="Imagem de saída ou pasta de saída, caso use --batch.",
    )

    parser.add_argument(
        "--threshold",
        type=int,
        default=245,
        help="Sensibilidade da remoção. Menor = mais agressivo. Padrão: 245.",
    )

    parser.add_argument(
        "--batch",
        action="store_true",
        help="Processa uma pasta inteira.",
    )

    parser.add_argument(
        "--no-smooth",
        action="store_true",
        help="Desativa suavização das bordas.",
    )

    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    smooth_edges = not args.no_smooth

    if args.batch:
        if args.output is None:
            output_folder = "imagens_sem_fundo"
        else:
            output_folder = args.output

        generated = process_folder(
            input_folder=args.input,
            output_folder=output_folder,
            threshold=args.threshold,
            smooth_edges=smooth_edges,
        )

        print(f"\nConcluído. Arquivos gerados: {len(generated)}")
        return

    output = remove_white_background(
        input_path=args.input,
        output_path=args.output,
        threshold=args.threshold,
        smooth_edges=smooth_edges,
    )

    print(f"Imagem salva em: {output}")


if __name__ == "__main__":
    main()
