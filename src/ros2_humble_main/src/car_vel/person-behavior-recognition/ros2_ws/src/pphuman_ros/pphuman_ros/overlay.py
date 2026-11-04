"""Single Pillow conversion per preview; fonts are loaded once."""
from pathlib import Path
from collections import OrderedDict

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

EDGES = [(5, 6), (5, 7), (7, 9), (6, 8), (8, 10), (5, 11), (6, 12),
         (11, 12), (11, 13), (13, 15), (12, 14), (14, 16), (0, 1), (0, 2)]
STATUS_TEXT = {
    'valid': '运行', 'warming': '预热', 'partial': '部分结果',
    'partial_or_warming': '部分结果／预热', 'unknown': '未知',
    'waiting_for_image': '等待图像', 'loading': '模型加载中',
    'stream_lost': '图像断流', 'result_expired': '结果已过期',
    'worker_failed': '模型异常', 'error': '异常',
    'fixture_only': '合成测试（无真实推理）',
}


def action_text(action, positive):
    state = action.get('state', 'unknown')
    if state == 'warming':
        return '预热中'
    if state != 'valid':
        if action.get('reason') in ('insufficient_visible_keypoints', 'incomplete_body', 'insufficient_lower_body'):
            return '身体不完整'
        if action.get('reason') == 'insufficient_visible_joints':
            return '关键点不足'
        return '未知'
    label = action.get('label')
    # A descent-to-lying candidate cannot establish whether the fall was accidental.
    text = ('疑似跌倒' if label == 'fall' else '倒地姿态' if label == 'lying'
            else positive if label in ('smoking', 'fight')
            else '未检出' if label == 'other' else '未知')
    if label == 'other' and positive == '跌倒':
        text = {'sitting': '坐姿', 'standing': '站姿', 'bending': '弯腰'}.get(action.get('posture'), text)
    score = action.get('score')
    return text + (f' {score:.2f}' if isinstance(score, (int, float)) else '') + ('' if action.get('updated') else ' [缓存]')


def label_position(size, preferred, canvas_size, occupied, box, gap=6):
    """Try vertical gaps in the original column before moving beside the box."""
    width, height = size
    max_x, max_y = canvas_size[0] - width, canvas_size[1] - height
    clamp_x = lambda x: max(0, min(int(x), max_x))
    clamp_y = lambda y: max(0, min(int(y), max_y))
    x, y = clamp_x(preferred[0]), clamp_y(preferred[1])
    xs = list(dict.fromkeys(clamp_x(v) for v in
               (x, box[2] - width, box[0] - width - gap, box[2] + gap, 0, max_x)))
    ys = {y, 0, max_y}
    for left, top, right, bottom in occupied:
        ys.update((clamp_y(top - height - gap), clamp_y(bottom + gap)))
    ys = sorted(ys, key=lambda value: (abs(value - y), value))
    best, best_overlap = (x, y), float('inf')
    for left in xs:
        for top in ys:
            overlap = sum(max(0, min(left + width + gap, r) - max(left - gap, l)) *
                          max(0, min(top + height + gap, b) - max(top - gap, t))
                          for l, t, r, b in occupied)
            if overlap == 0:
                return left, top
            if overlap < best_overlap:
                best, best_overlap = (left, top), overlap
    # A physically overfull/tiny canvas cannot fit every label. Remain in bounds.
    return best


class Overlay:
    def __init__(self, font_path):
        candidates = [font_path, '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc']
        found = next((p for p in candidates if p and Path(p).is_file()), None)
        if not found:
            raise RuntimeError('中文字体不存在，请配置 font_path 为 NotoSansCJK 字体')
        self.font_path = found
        self.font = ImageFont.truetype(found, 22)
        self.fonts = {22: self.font}
        self.label_cache = OrderedDict()

    def label_tile(self, text, width, height):
        key = (text, width, height)
        cached = self.label_cache.get(key)
        if cached is not None:
            self.label_cache.move_to_end(key)
            return cached
        draw = ImageDraw.Draw(Image.new('RGB', (1, 1)))
        font = self.font
        bounds = draw.multiline_textbbox((0, 0), text, font=font, spacing=2)
        text_width, text_height = bounds[2] - bounds[0], bounds[3] - bounds[1]
        # Unusually small input images also keep the whole label inside the frame.
        if text_width > width - 8 or text_height > height - 8:
            size = max(1, int(22 * min(max(1, width - 8) / max(1, text_width),
                                        max(1, height - 8) / max(1, text_height))))
            if size not in self.fonts:
                self.fonts[size] = ImageFont.truetype(self.font_path, size)
            font = self.fonts[size]
            bounds = draw.multiline_textbbox((0, 0), text, font=font, spacing=2)
            text_width, text_height = bounds[2] - bounds[0], bounds[3] - bounds[1]
        tile = Image.new('RGB', (text_width + 1, text_height + 1), (15, 15, 15))
        ImageDraw.Draw(tile).multiline_text((-bounds[0], -bounds[1]), text,
            fill=(255, 245, 230), font=font, spacing=2)
        self.label_cache[key] = tile
        if len(self.label_cache) > 128:
            self.label_cache.popitem(last=False)
        return tile

    def draw_label(self, canvas, x, y, text, bottom_margin=0):
        width, height = canvas.size
        height = max(1, height - bottom_margin)
        tile = self.label_tile(text, width, height)
        left = max(0, min(x, width - tile.width))
        top = max(0, min(y, height - tile.height))
        canvas.paste(tile, (left, top))
        return left, top, left + tile.width, top + tile.height

    def render(self, frame, result):
        image = frame.copy()
        labels = []
        for person in result.get('persons', []):
            box = person.get('bbox', [])
            if len(box) != 4:
                continue
            x1, y1, x2, y2 = [int(float(x)) for x in box]
            cv2.rectangle(image, (x1, y1), (x2, y2), (40, 220, 40), 2)
            points = person.get('keypoints')
            if isinstance(points, list) and len(points) >= 17:
                for a, b in EDGES:
                    pa, pb = points[a], points[b]
                    if len(pa) >= 2 and len(pb) >= 2 and (len(pa) < 3 or pa[2] > .2) and (len(pb) < 3 or pb[2] > .2):
                        cv2.line(image, tuple(map(int, pa[:2])), tuple(map(int, pb[:2])), (255, 200, 0), 2)
            actions = person.get('actions', {})
            labels.append(((x1, y1, x2, y2),
                           f"ID {person.get('track_id', '?')} 吸烟：{action_text(actions.get('smoking', {}), '吸烟')}\n"
                           f"跌倒：{action_text(actions.get('fall', {}), '跌倒')}"))
        canvas = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        scene = result.get('scene', {}).get('fight', {})
        title_text = (f"全场景打架：{action_text(scene, '打架')}\n"
                      f"状态：{STATUS_TEXT.get(result.get('status'), '未知')}")
        title = self.draw_label(canvas, 8, 8, title_text)
        occupied = [title]
        diagnostics = result.get('preview_diagnostics')
        if diagnostics:
            age = diagnostics.get('result_age_ms')
            # The HUD says "approximately"; precise ages remain in preview_status.
            # 50ms display buckets make the text cache useful at a 20Hz image rate.
            age_text = f'{round(age / 50) * 50:.0f}ms' if age is not None else '等待'
            state_text = '' if diagnostics.get('overlay_applied') else '（标注不可用）'
            hud_text = (f"画面 {diagnostics.get('preview_hz', 0):.0f}Hz 识别 {diagnostics.get('processing_hz', 0):.1f}Hz "
                        f"结果年龄约 {age_text}{state_text}")
            self.draw_label(canvas, 8, canvas.height - 32, hud_text)
            occupied.append((0, canvas.height - 36, canvas.width, canvas.height))
        available_height = max(1, canvas.height - (36 if diagnostics else 0))
        placed = []
        for box, text in labels:
            tile = self.label_tile(text, canvas.width, available_height)
            left, top = label_position(tile.size, (box[0], box[1] - tile.height - 6),
                                       (canvas.width, available_height), occupied, box)
            rect = (left, top, left + tile.width, top + tile.height)
            occupied.append(rect)
            placed.append((box, tile, rect))
        draw = ImageDraw.Draw(canvas)
        for box, tile, (left, top, right, bottom) in placed:
            # Draw leaders before tiles so they cannot strike through any text.
            anchor = (max(box[0], min((left + right) // 2, box[2])),
                      max(box[1], min((top + bottom) // 2, box[3])))
            edge = (max(left, min(anchor[0], right)), max(top, min(anchor[1], bottom)))
            if abs(edge[0] - anchor[0]) + abs(edge[1] - anchor[1]) > 6:
                draw.line((edge, anchor), fill=(40, 220, 40), width=1)
        for _, tile, (left, top, _, _) in placed:
            canvas.paste(tile, (left, top))
        self.draw_label(canvas, 8, 8, title_text)
        if diagnostics:
            self.draw_label(canvas, 8, canvas.height - 32, hud_text)
        return cv2.cvtColor(np.asarray(canvas), cv2.COLOR_RGB2BGR)

    def unavailable(self, shape=(360, 640, 3), text='图像已断流／结果已过期'):
        canvas = Image.new('RGB', (shape[1], shape[0]), (20, 20, 20))
        ImageDraw.Draw(canvas).text((20, 30), text, font=self.font, fill=(255, 200, 90))
        return cv2.cvtColor(np.asarray(canvas), cv2.COLOR_RGB2BGR)
