#!/usr/bin/env python3
"""Generate ERC landmark textures and Unity materials from Assets/ArUco SVGs."""

from __future__ import annotations

import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

from PIL import Image, ImageDraw, ImageFont


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SOURCE_DIR = PROJECT_ROOT / "Assets" / "ArUco"
OUTPUT_DIR = SOURCE_DIR / "LandmarkTextures"

# 2.8 pixels/mm makes the 150 mm, 7-module marker exactly 420 px (60 px/module).
PX_PER_MM = 2.8
FACE_WIDTH_MM = 250
FACE_HEIGHT_MM = 320
LABEL_X_MM = 50
LABEL_Y_MM = 20
LABEL_WIDTH_MM = 150
LABEL_HEIGHT_MM = 70
MARKER_X_MM = 50
MARKER_Y_MM = 120
MARKER_SIZE_MM = 150
LABEL_BORDER_MM = 0.7
FONT_SIZE_MM = 71
FONT_CANDIDATES = (
    Path("/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf"),
    Path("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"),
    Path("C:/Windows/Fonts/arialbd.ttf"),
    Path("/Library/Fonts/Arial Bold.ttf"),
)


def px(mm: float) -> int:
    return round(mm * PX_PER_MM)


def find_font() -> Path:
    for candidate in FONT_CANDIDATES:
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(
        "No supported bold sans-serif font was found. "
        f"Checked: {', '.join(str(path) for path in FONT_CANDIDATES)}"
    )


def unity_guid(relative_path: Path) -> str:
    """Return a stable Unity-compatible 32-character GUID for a generated asset."""
    key = f"erc-landmark-textures/{relative_path.as_posix()}".encode("utf-8")
    return hashlib.md5(key).hexdigest()


def draw_marker(draw: ImageDraw.ImageDraw, svg_path: Path) -> None:
    root = ET.parse(svg_path).getroot()
    view_box = [float(value) for value in root.attrib["viewBox"].split()]
    _, _, view_width, view_height = view_box
    marker_left = px(MARKER_X_MM)
    marker_top = px(MARKER_Y_MM)
    marker_size = px(MARKER_SIZE_MM)

    for rect in root.findall("{http://www.w3.org/2000/svg}rect"):
        x = float(rect.attrib.get("x", 0))
        y = float(rect.attrib.get("y", 0))
        width = float(rect.attrib["width"])
        height = float(rect.attrib["height"])
        fill = rect.attrib.get("fill", "black").lower()
        color = (255, 255, 255) if fill == "white" else (0, 0, 0)

        x0 = marker_left + round(x / view_width * marker_size)
        y0 = marker_top + round(y / view_height * marker_size)
        x1 = marker_left + round((x + width) / view_width * marker_size)
        y1 = marker_top + round((y + height) / view_height * marker_size)
        draw.rectangle((x0, y0, x1 - 1, y1 - 1), fill=color)


def draw_number(draw: ImageDraw.ImageDraw, number: int, font_path: Path) -> None:
    x0 = px(LABEL_X_MM)
    y0 = px(LABEL_Y_MM)
    x1 = x0 + px(LABEL_WIDTH_MM)
    y1 = y0 + px(LABEL_HEIGHT_MM)
    border = max(1, px(LABEL_BORDER_MM))
    draw.rectangle((x0, y0, x1 - 1, y1 - 1), outline=(0, 0, 0), width=border)

    font = ImageFont.truetype(str(font_path), px(FONT_SIZE_MM))
    text = str(number)
    bounds = draw.textbbox((0, 0), text, font=font)
    text_width = bounds[2] - bounds[0]
    text_height = bounds[3] - bounds[1]
    text_x = x0 + (x1 - x0 - text_width) / 2 - bounds[0]
    text_y = y0 + (y1 - y0 - text_height) / 2 - bounds[1]
    draw.text((round(text_x), round(text_y)), text, font=font, fill=(0, 0, 0))


def texture_meta(guid: str) -> str:
    return f"""fileFormatVersion: 2
guid: {guid}
TextureImporter:
  internalIDToNameTable: []
  externalObjects: {{}}
  serializedVersion: 13
  mipmaps:
    mipMapMode: 0
    enableMipMap: 0
    sRGBTexture: 1
    linearTexture: 0
    fadeOut: 0
    borderMipMap: 0
    mipMapsPreserveCoverage: 0
    alphaTestReferenceValue: 0.5
    mipMapFadeDistanceStart: 1
    mipMapFadeDistanceEnd: 3
  bumpmap:
    convertToNormalMap: 0
    externalNormalMap: 0
    heightScale: 0.25
    normalMapFilter: 0
    flipGreenChannel: 0
  isReadable: 0
  streamingMipmaps: 0
  streamingMipmapsPriority: 0
  vTOnly: 0
  ignoreMipmapLimit: 0
  grayScaleToAlpha: 0
  generateCubemap: 6
  cubemapConvolution: 0
  seamlessCubemap: 0
  textureFormat: 1
  maxTextureSize: 2048
  textureSettings:
    serializedVersion: 2
    filterMode: 0
    aniso: 1
    mipBias: 0
    wrapU: 1
    wrapV: 1
    wrapW: 1
  nPOTScale: 0
  lightmap: 0
  compressionQuality: 100
  spriteMode: 0
  spriteExtrude: 1
  spriteMeshType: 1
  alignment: 0
  spritePivot: {{x: 0.5, y: 0.5}}
  spritePixelsToUnits: 100
  spriteBorder: {{x: 0, y: 0, z: 0, w: 0}}
  spriteGenerateFallbackPhysicsShape: 1
  alphaUsage: 0
  alphaIsTransparency: 0
  spriteTessellationDetail: -1
  textureType: 0
  textureShape: 1
  singleChannelComponent: 0
  flipbookRows: 1
  flipbookColumns: 1
  maxTextureSizeSet: 0
  compressionQualitySet: 0
  textureFormatSet: 0
  ignorePngGamma: 0
  applyGammaDecoding: 0
  swizzle: 50462976
  cookieLightType: 0
  platformSettings:
  - serializedVersion: 3
    buildTarget: DefaultTexturePlatform
    maxTextureSize: 2048
    resizeAlgorithm: 0
    textureFormat: -1
    textureCompression: 0
    compressionQuality: 100
    crunchedCompression: 0
    allowsAlphaSplitting: 0
    overridden: 0
    ignorePlatformSupport: 0
    androidETC2FallbackOverride: 0
    forceMaximumCompressionQuality_BC6H_BC7: 0
  spriteSheet:
    serializedVersion: 2
    sprites: []
    outline: []
    physicsShape: []
    bones: []
    spriteID: 
    internalID: 0
    vertices: []
    indices: 
    edges: []
    weights: []
    secondaryTextures: []
    nameFileIdTable: {{}}
  mipmapLimitGroupName: 
  pSDRemoveMatte: 0
  userData: ERC landmark face: 250x320mm; marker: 150x150mm
  assetBundleName: 
  assetBundleVariant: 
"""


def material_yaml(name: str, texture_guid: str) -> str:
    return f"""%YAML 1.1
%TAG !u! tag:unity3d.com,2011:
--- !u!21 &2100000
Material:
  serializedVersion: 8
  m_ObjectHideFlags: 0
  m_CorrespondingSourceObject: {{fileID: 0}}
  m_PrefabInstance: {{fileID: 0}}
  m_PrefabAsset: {{fileID: 0}}
  m_Name: {name}
  m_Shader: {{fileID: 46, guid: 0000000000000000f000000000000000, type: 0}}
  m_Parent: {{fileID: 0}}
  m_ModifiedSerializedProperties: 0
  m_ValidKeywords: []
  m_InvalidKeywords: []
  m_LightmapFlags: 4
  m_EnableInstancingVariants: 1
  m_DoubleSidedGI: 0
  m_CustomRenderQueue: -1
  stringTagMap: {{}}
  disabledShaderPasses: []
  m_LockedProperties: 
  m_SavedProperties:
    serializedVersion: 3
    m_TexEnvs:
    - _MainTex:
        m_Texture: {{fileID: 2800000, guid: {texture_guid}, type: 3}}
        m_Scale: {{x: 1, y: 1}}
        m_Offset: {{x: 0, y: 0}}
    m_Ints: []
    m_Floats:
    - _Glossiness: 0
    - _Metallic: 0
    - _Mode: 0
    - _SmoothnessTextureChannel: 0
    m_Colors:
    - _Color: {{r: 1, g: 1, b: 1, a: 1}}
    - _EmissionColor: {{r: 0, g: 0, b: 0, a: 1}}
  m_BuildTextureStacks: []
"""


def simple_meta(guid: str) -> str:
    return f"""fileFormatVersion: 2
guid: {guid}
NativeFormatImporter:
  externalObjects: {{}}
  mainObjectFileID: 2100000
  userData: 
  assetBundleName: 
  assetBundleVariant: 
"""


def generate() -> None:
    font_path = find_font()

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    for marker_id in range(51, 66):
        number = marker_id - 50
        source = SOURCE_DIR / f"5x5_1000-{marker_id}.svg"
        if not source.is_file():
            raise FileNotFoundError(f"Missing marker SVG: {source}")

        stem = f"Landmark_{number:02d}_ID{marker_id}"
        png_path = OUTPUT_DIR / f"{stem}.png"
        mat_path = OUTPUT_DIR / f"{stem}.mat"
        texture_guid = unity_guid(png_path.relative_to(PROJECT_ROOT))
        material_guid = unity_guid(mat_path.relative_to(PROJECT_ROOT))

        image = Image.new("RGB", (px(FACE_WIDTH_MM), px(FACE_HEIGHT_MM)), "white")
        draw = ImageDraw.Draw(image)
        draw_number(draw, number, font_path)
        draw_marker(draw, source)
        image.save(png_path, format="PNG", optimize=True)

        png_path.with_suffix(".png.meta").write_text(texture_meta(texture_guid), encoding="utf-8")
        mat_path.write_text(material_yaml(stem, texture_guid), encoding="utf-8")
        mat_path.with_suffix(".mat.meta").write_text(simple_meta(material_guid), encoding="utf-8")

        print(f"Generated {png_path.relative_to(PROJECT_ROOT)}")


if __name__ == "__main__":
    generate()
