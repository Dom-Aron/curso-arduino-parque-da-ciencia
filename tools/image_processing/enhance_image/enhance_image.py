"""
enhance_image.py

Melhora a qualidade visual de imagens usando técnicas clássicas de processamento:

- Redimensionamento por escala, largura ou altura.
- Interpolação de alta qualidade.
- Redução leve de ruído.
- Aumento controlado de nitidez.
- Ajuste opcional de contraste local com CLAHE.
- Preservação de transparência quando a imagem original possui canal alfa.
- Processamento individual ou em lote.
- Modo helper interativo.
- Dry-run para simular sem salvar arquivos.
- Evita sobrescrever arquivos por padrão.

Observação importante:
Este script melhora nitidez percebida e aumenta resolução em pixels, mas não cria
detalhes reais como um modelo de super-resolução por IA faria. Para materiais de
aula, README, slides e documentação, normalmente já resolve muito bem.

Dependências:
    pip install opencv-python numpy pillow

Exemplos:
    python enhance_image.py
    python enhance_image.py --scale 2
    python enhance_image.py --scale 3 --sharpen 1.2
    python enhance_image.py -i input/lcd.png -o output/lcd_melhorado.png
    python enhance_image.py -i input -o output --recursive
    python enhance_image.py --width 1200
    python enhance_image.py --height 800
    python enhance_image.py --contrast
    python enhance_image.py --helper
    python enhance_image.py --examples
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
from PIL import Image


VALID_EXTENSIONS = {".png", ".jpg", ".jpeg", ".webp", ".bmp", ".tif", ".tiff"}


def get_script_dir() -> Path:
    """Retorna a pasta em que este script está salvo."""
    return Path(__file__).resolve().parent


def ensure_default_folders(base_dir: Path) -> tuple[Path, Path]:
    """Cria as pastas padrão input/ e output/ ao lado do script."""
    input_dir = base_dir / "input"
    output_dir = base_dir / "output"

    input_dir.mkdir(parents=True, exist_ok=True)
    output_dir.mkdir(parents=True, exist_ok=True)

    return input_dir, output_dir


def is_valid_image(path: Path) -> bool:
    """Verifica se o arquivo possui uma extensão de imagem suportada."""
    return path.is_file() and path.suffix.lower() in VALID_EXTENSIONS


def list_images(input_path: Path, recursive: bool = False) -> list[Path]:
    """Lista imagens a partir de uma imagem única ou de uma pasta."""
    if input_path.is_file():
        return [input_path] if is_valid_image(input_path) else []

    if not input_path.exists():
        return []

    iterator = input_path.rglob("*") if recursive else input_path.iterdir()
    return sorted(path for path in iterator if is_valid_image(path))


def avoid_overwrite(path: Path) -> Path:
    """Evita sobrescrever arquivos existentes."""
    if not path.exists():
        return path

    parent = path.parent
    stem = path.stem
    suffix = path.suffix
    counter = 1

    while True:
        candidate = parent / f"{stem}_{counter:03d}{suffix}"
        if not candidate.exists():
            return candidate
        counter += 1


def build_output_path(
    image_path: Path,
    input_root: Path,
    output_root: Path,
    suffix: str = "_enhanced",
    output_ext: str = ".png",
) -> Path:
    """Monta o caminho de saída, preservando subpastas quando houver."""
    if input_root.is_file():
        relative_parent = Path()
    else:
        try:
            relative_parent = image_path.parent.relative_to(input_root)
        except ValueError:
            relative_parent = Path()

    if not output_ext.startswith("."):
        output_ext = f".{output_ext}"

    return output_root / relative_parent / f"{image_path.stem}{suffix}{output_ext}"


def read_image_preserve_alpha(path: Path) -> tuple[np.ndarray, np.ndarray | None]:
    """
    Lê uma imagem preservando o canal alfa, quando existir.

    Returns
    -------
    tuple[np.ndarray, np.ndarray | None]
        Imagem BGR e canal alfa opcional.
    """
    image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)

    if image is None:
        raise FileNotFoundError(f"Não foi possível abrir a imagem: {path}")

    if image.ndim == 2:
        bgr = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        return bgr, None

    if image.shape[2] == 4:
        bgr = image[:, :, :3]
        alpha = image[:, :, 3]
        return bgr, alpha

    return image, None


def calculate_target_size(
    width: int,
    height: int,
    scale: float | None = None,
    target_width: int | None = None,
    target_height: int | None = None,
) -> tuple[int, int]:
    """
    Calcula o tamanho final da imagem.

    Prioridade:
    1. target_width e target_height juntos;
    2. target_width mantendo proporção;
    3. target_height mantendo proporção;
    4. scale;
    5. tamanho original.
    """
    if target_width and target_height:
        return target_width, target_height

    if target_width:
        factor = target_width / width
        return target_width, max(1, int(round(height * factor)))

    if target_height:
        factor = target_height / height
        return max(1, int(round(width * factor))), target_height

    if scale:
        return max(1, int(round(width * scale))), max(1, int(round(height * scale)))

    return width, height


def choose_interpolation(old_width: int, old_height: int, new_width: int, new_height: int) -> int:
    """
    Escolhe uma interpolação adequada.

    Para aumentar, usa Lanczos.
    Para reduzir, usa Area.
    """
    if new_width >= old_width or new_height >= old_height:
        return cv2.INTER_LANCZOS4

    return cv2.INTER_AREA


def apply_denoise(bgr: np.ndarray, strength: float) -> np.ndarray:
    """
    Aplica redução de ruído.

    strength:
        0.0 desativa.
        3.0 a 7.0 costuma ser leve/moderado.
    """
    if strength <= 0:
        return bgr

    h = float(strength)
    return cv2.fastNlMeansDenoisingColored(
        bgr,
        None,
        h=h,
        hColor=h,
        templateWindowSize=7,
        searchWindowSize=21,
    )


def apply_clahe_contrast(bgr: np.ndarray, clip_limit: float = 2.0, tile_grid_size: int = 8) -> np.ndarray:
    """
    Aplica contraste local usando CLAHE no canal de luminância.
    """
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    l_channel, a_channel, b_channel = cv2.split(lab)

    clahe = cv2.createCLAHE(
        clipLimit=clip_limit,
        tileGridSize=(tile_grid_size, tile_grid_size),
    )

    l_enhanced = clahe.apply(l_channel)
    lab_enhanced = cv2.merge([l_enhanced, a_channel, b_channel])

    return cv2.cvtColor(lab_enhanced, cv2.COLOR_LAB2BGR)


def apply_sharpen(bgr: np.ndarray, amount: float) -> np.ndarray:
    """
    Aplica nitidez usando unsharp mask.

    amount:
        0.0 desativa.
        0.6 é leve.
        1.0 é moderado.
        1.5 é forte.
    """
    if amount <= 0:
        return bgr

    blurred = cv2.GaussianBlur(bgr, (0, 0), sigmaX=1.2)
    sharpened = cv2.addWeighted(bgr, 1.0 + amount, blurred, -amount, 0)

    return np.clip(sharpened, 0, 255).astype(np.uint8)


def add_alpha_if_needed(bgr: np.ndarray, alpha: np.ndarray | None) -> np.ndarray:
    """Recombina a imagem BGR com o canal alfa, quando existir."""
    if alpha is None:
        return bgr

    return cv2.merge([bgr[:, :, 0], bgr[:, :, 1], bgr[:, :, 2], alpha])


def save_image_with_optional_dpi(
    image: np.ndarray,
    output_path: Path,
    dpi: int | None = None,
    jpeg_quality: int = 95,
) -> None:
    """
    Salva a imagem.

    Usa Pillow quando DPI for informado, porque ele lida melhor com metadados.
    """
    output_path.parent.mkdir(parents=True, exist_ok=True)

    suffix = output_path.suffix.lower()

    if image.ndim == 3 and image.shape[2] == 4:
        rgba = cv2.cvtColor(image, cv2.COLOR_BGRA2RGBA)
        pil_image = Image.fromarray(rgba)
    else:
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        pil_image = Image.fromarray(rgb)

    save_kwargs = {}

    if dpi is not None:
        save_kwargs["dpi"] = (dpi, dpi)

    if suffix in {".jpg", ".jpeg"}:
        if pil_image.mode == "RGBA":
            pil_image = pil_image.convert("RGB")
        save_kwargs["quality"] = jpeg_quality
        save_kwargs["subsampling"] = 0

    pil_image.save(output_path, **save_kwargs)


def enhance_image(
    input_path: str | Path,
    output_path: str | Path,
    scale: float | None = 2.0,
    target_width: int | None = None,
    target_height: int | None = None,
    denoise: float = 0.0,
    sharpen: float = 0.8,
    contrast: bool = False,
    contrast_clip: float = 2.0,
    force: bool = False,
    dpi: int | None = None,
    jpeg_quality: int = 95,
) -> Path:
    """
    Melhora a qualidade visual de uma imagem e salva o resultado.
    """
    input_file = Path(input_path)
    output_file = Path(output_path)

    if not force:
        output_file = avoid_overwrite(output_file)

    bgr, alpha = read_image_preserve_alpha(input_file)

    old_height, old_width = bgr.shape[:2]
    new_width, new_height = calculate_target_size(
        width=old_width,
        height=old_height,
        scale=scale,
        target_width=target_width,
        target_height=target_height,
    )

    interpolation = choose_interpolation(old_width, old_height, new_width, new_height)

    bgr = cv2.resize(
        bgr,
        (new_width, new_height),
        interpolation=interpolation,
    )

    if alpha is not None:
        alpha = cv2.resize(
            alpha,
            (new_width, new_height),
            interpolation=cv2.INTER_LANCZOS4,
        )

    bgr = apply_denoise(bgr, denoise)

    if contrast:
        bgr = apply_clahe_contrast(bgr, clip_limit=contrast_clip)

    bgr = apply_sharpen(bgr, sharpen)

    final_image = add_alpha_if_needed(bgr, alpha)

    save_image_with_optional_dpi(
        image=final_image,
        output_path=output_file,
        dpi=dpi,
        jpeg_quality=jpeg_quality,
    )

    return output_file


def process_images(
    input_path: Path,
    output_path: Path,
    scale: float | None,
    target_width: int | None,
    target_height: int | None,
    denoise: float,
    sharpen: float,
    contrast: bool,
    contrast_clip: float,
    recursive: bool,
    force: bool,
    dry_run: bool,
    suffix: str,
    output_ext: str,
    dpi: int | None,
    jpeg_quality: int,
) -> list[tuple[Path, Path]]:
    """Processa uma imagem ou uma pasta de imagens."""
    images = list_images(input_path, recursive=recursive)
    results: list[tuple[Path, Path]] = []

    if not images:
        print(f"Nenhuma imagem encontrada em: {input_path}")
        return results

    for image_path in images:
        output_file = build_output_path(
            image_path=image_path,
            input_root=input_path,
            output_root=output_path,
            suffix=suffix,
            output_ext=output_ext,
        )

        if dry_run:
            print(f"[DRY-RUN] {image_path} -> {output_file}")
            results.append((image_path, output_file))
            continue

        generated_file = enhance_image(
            input_path=image_path,
            output_path=output_file,
            scale=scale,
            target_width=target_width,
            target_height=target_height,
            denoise=denoise,
            sharpen=sharpen,
            contrast=contrast,
            contrast_clip=contrast_clip,
            force=force,
            dpi=dpi,
            jpeg_quality=jpeg_quality,
        )

        results.append((image_path, generated_file))
        print(f"OK: {image_path.name} -> {generated_file.name}")

    return results


def parse_bool_answer(value: str, default: bool = False) -> bool:
    """Interpreta respostas simples de sim/não."""
    value = value.strip().lower()

    if value == "":
        return default

    if value in {"s", "sim", "y", "yes"}:
        return True

    if value in {"n", "nao", "não", "no"}:
        return False

    return default


def prompt_path(message: str, default: Path) -> Path:
    """Lê um caminho com valor padrão."""
    answer = input(f"{message} [{default}]: ").strip()
    return Path(answer) if answer else default


def prompt_float(message: str, default: float) -> float:
    """Lê um número decimal com valor padrão."""
    answer = input(f"{message} [{default}]: ").strip()

    if not answer:
        return default

    try:
        return float(answer.replace(",", "."))
    except ValueError:
        print("Valor inválido. Usando o padrão.")
        return default


def prompt_optional_int(message: str, default: int | None = None) -> int | None:
    """Lê um inteiro opcional."""
    default_text = "" if default is None else str(default)
    answer = input(f"{message} [{default_text}]: ").strip()

    if not answer:
        return default

    try:
        return int(answer)
    except ValueError:
        print("Valor inválido. Ignorando.")
        return default


def run_helper(default_input: Path, default_output: Path) -> argparse.Namespace:
    """Executa um modo guiado/interativo."""
    print("\n=== Helper: melhoria de qualidade de imagem ===")
    print("Pressione Enter para aceitar os valores padrão.\n")

    input_path = prompt_path("Caminho de entrada (arquivo ou pasta)", default_input)
    output_path = prompt_path("Caminho de saída (pasta ou arquivo)", default_output)

    scale = prompt_float("Escala de ampliação", 2.0)

    width = prompt_optional_int("Largura final em pixels, opcional", None)
    height = prompt_optional_int("Altura final em pixels, opcional", None)

    denoise = prompt_float("Redução de ruído: 0 desliga, 3 a 7 é leve/moderado", 0.0)
    sharpen = prompt_float("Nitidez: 0 desliga, 0.8 leve/moderado, 1.5 forte", 0.8)

    contrast = parse_bool_answer(input("Aplicar contraste local CLAHE? [n]: "), default=False)
    contrast_clip = prompt_float("Força do contraste CLAHE", 2.0) if contrast else 2.0

    recursive = parse_bool_answer(input("Processar subpastas? [n]: "), default=False)
    force = parse_bool_answer(input("Sobrescrever arquivos existentes? [n]: "), default=False)
    dry_run = parse_bool_answer(input("Executar apenas simulação (dry-run)? [n]: "), default=False)

    dpi = prompt_optional_int("DPI opcional para metadados da imagem", None)

    suffix_answer = input("Sufixo dos arquivos de saída [_enhanced]: ").strip()
    suffix = suffix_answer if suffix_answer else "_enhanced"

    output_ext_answer = input("Extensão de saída [.png]: ").strip()
    output_ext = output_ext_answer if output_ext_answer else ".png"

    return argparse.Namespace(
        input_pos=None,
        output_pos=None,
        input_opt=str(input_path),
        output_opt=str(output_path),
        scale=scale,
        width=width,
        height=height,
        denoise=denoise,
        sharpen=sharpen,
        contrast=contrast,
        contrast_clip=contrast_clip,
        recursive=recursive,
        force=force,
        dry_run=dry_run,
        helper=True,
        examples=False,
        suffix=suffix,
        output_ext=output_ext,
        dpi=dpi,
        jpeg_quality=95,
    )


def print_examples() -> None:
    """Mostra exemplos práticos de uso."""
    print("\nExemplos de uso\n")
    print("1) Uso padrão: processa input/ e salva em output/")
    print("   python enhance_image.py\n")
    print("2) Dobrar resolução:")
    print("   python enhance_image.py --scale 2\n")
    print("3) Triplicar resolução com mais nitidez:")
    print("   python enhance_image.py --scale 3 --sharpen 1.2\n")
    print("4) Definir largura final mantendo proporção:")
    print("   python enhance_image.py --width 1200\n")
    print("5) Definir altura final mantendo proporção:")
    print("   python enhance_image.py --height 800\n")
    print("6) Imagem única:")
    print("   python enhance_image.py -i input/lcd.png -o output/lcd_melhorado.png\n")
    print("7) Pasta inteira com subpastas:")
    print("   python enhance_image.py -i input -o output --recursive\n")
    print("8) Aplicar contraste local:")
    print("   python enhance_image.py --contrast\n")
    print("9) Redução leve de ruído:")
    print("   python enhance_image.py --denoise 4\n")
    print("10) Modo guiado:")
    print("   python enhance_image.py --helper\n")


def build_parser() -> argparse.ArgumentParser:
    """Cria o parser de argumentos de linha de comando."""
    parser = argparse.ArgumentParser(
        description="Melhora qualidade, escala e nitidez de imagens."
    )

    parser.add_argument("input_pos", nargs="?", help="Imagem ou pasta de entrada.")
    parser.add_argument("output_pos", nargs="?", help="Imagem ou pasta de saída.")

    parser.add_argument("-i", "--input", dest="input_opt", help="Imagem ou pasta de entrada.")
    parser.add_argument("-o", "--output", dest="output_opt", help="Imagem ou pasta de saída.")

    parser.add_argument(
        "--scale",
        type=float,
        default=2.0,
        help="Fator de escala. Padrão: 2. Use 1 para manter o tamanho.",
    )

    parser.add_argument(
        "--width",
        type=int,
        default=None,
        help="Largura final em pixels, mantendo proporção se --height não for usado.",
    )

    parser.add_argument(
        "--height",
        type=int,
        default=None,
        help="Altura final em pixels, mantendo proporção se --width não for usado.",
    )

    parser.add_argument(
        "--denoise",
        type=float,
        default=0.0,
        help="Redução de ruído. 0 desliga. Valores úteis: 3 a 7.",
    )

    parser.add_argument(
        "--sharpen",
        type=float,
        default=0.8,
        help="Nitidez. 0 desliga. 0.8 leve/moderado; 1.5 forte.",
    )

    parser.add_argument(
        "--contrast",
        action="store_true",
        help="Aplica contraste local CLAHE.",
    )

    parser.add_argument(
        "--contrast-clip",
        type=float,
        default=2.0,
        help="Força do contraste CLAHE. Padrão: 2.0.",
    )

    parser.add_argument(
        "--recursive",
        action="store_true",
        help="Processa subpastas.",
    )

    parser.add_argument(
        "--force",
        action="store_true",
        help="Sobrescreve arquivos existentes.",
    )

    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Mostra o que seria feito sem salvar arquivos.",
    )

    parser.add_argument(
        "--suffix",
        default="_enhanced",
        help="Sufixo usado nos arquivos gerados. Padrão: _enhanced.",
    )

    parser.add_argument(
        "--output-ext",
        default=".png",
        help="Extensão/formato de saída. Padrão: .png.",
    )

    parser.add_argument(
        "--dpi",
        type=int,
        default=None,
        help="DPI opcional salvo nos metadados da imagem.",
    )

    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=95,
        help="Qualidade JPEG se a saída for JPG/JPEG. Padrão: 95.",
    )

    parser.add_argument(
        "--helper",
        action="store_true",
        help="Abre o modo guiado/interativo.",
    )

    parser.add_argument(
        "--examples",
        action="store_true",
        help="Mostra exemplos de uso e sai.",
    )

    return parser


def resolve_paths(args: argparse.Namespace, default_input: Path, default_output: Path) -> tuple[Path, Path | None]:
    """
    Resolve prioridade de caminhos:
    1. flags -i/--input e -o/--output;
    2. argumentos posicionais;
    3. pastas padrão input/ e output/.
    """
    raw_input = args.input_opt if args.input_opt is not None else args.input_pos
    raw_output = args.output_opt if args.output_opt is not None else args.output_pos

    input_path = Path(raw_input) if raw_input else default_input
    output_path = Path(raw_output) if raw_output else None

    return input_path, output_path


def main() -> None:
    """Executa o programa."""
    parser = build_parser()
    args = parser.parse_args()

    base_dir = get_script_dir()
    default_input, default_output = ensure_default_folders(base_dir)

    if args.examples:
        print_examples()
        return

    if args.helper:
        args = run_helper(default_input, default_output)

    input_path, output_arg = resolve_paths(args, default_input, default_output)

    if not input_path.exists():
        print(f"Entrada não encontrada: {input_path}")
        print("Pastas padrão garantidas:")
        print(f"  input : {default_input}")
        print(f"  output: {default_output}")
        return

    is_single_image_mode = input_path.is_file()

    if is_single_image_mode:
        if output_arg:
            output_path = output_arg
        else:
            output_path = default_output / f"{input_path.stem}{args.suffix}{args.output_ext}"

        if args.dry_run:
            print(f"[DRY-RUN] {input_path} -> {output_path}")
            return

        generated_file = enhance_image(
            input_path=input_path,
            output_path=output_path,
            scale=args.scale,
            target_width=args.width,
            target_height=args.height,
            denoise=args.denoise,
            sharpen=args.sharpen,
            contrast=args.contrast,
            contrast_clip=args.contrast_clip,
            force=args.force,
            dpi=args.dpi,
            jpeg_quality=args.jpeg_quality,
        )

        print("\nResumo")
        print(f"Entrada          : {input_path}")
        print(f"Imagem melhorada : {generated_file}")
        print(f"Escala           : {args.scale}")
        print(f"Largura alvo     : {args.width}")
        print(f"Altura alvo      : {args.height}")
        print(f"Nitidez          : {args.sharpen}")
        print(f"Ruído            : {args.denoise}")
        print(f"Contraste CLAHE  : {'sim' if args.contrast else 'não'}")
        print(f"DPI              : {args.dpi}")
        return

    output_path = output_arg if output_arg else default_output

    results = process_images(
        input_path=input_path,
        output_path=output_path,
        scale=args.scale,
        target_width=args.width,
        target_height=args.height,
        denoise=args.denoise,
        sharpen=args.sharpen,
        contrast=args.contrast,
        contrast_clip=args.contrast_clip,
        recursive=args.recursive,
        force=args.force,
        dry_run=args.dry_run,
        suffix=args.suffix,
        output_ext=args.output_ext,
        dpi=args.dpi,
        jpeg_quality=args.jpeg_quality,
    )

    print("\nResumo")
    print(f"Entrada        : {input_path}")
    print(f"Saída          : {output_path}")
    print(f"Escala         : {args.scale}")
    print(f"Largura alvo   : {args.width}")
    print(f"Altura alvo    : {args.height}")
    print(f"Nitidez        : {args.sharpen}")
    print(f"Ruído          : {args.denoise}")
    print(f"Contraste      : {'sim' if args.contrast else 'não'}")
    print(f"Recursivo      : {'sim' if args.recursive else 'não'}")
    print(f"Dry-run        : {'sim' if args.dry_run else 'não'}")
    print(f"Sufixo         : {args.suffix}")
    print(f"Extensão saída : {args.output_ext}")
    print(f"DPI            : {args.dpi}")
    print(f"Processadas    : {len(results)} imagem(ns)")


if __name__ == "__main__":
    main()
