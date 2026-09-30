"""sheet.py MODEL TITLE LEFT_NOTE RIGHT_NOTE [crop x y w h]: 2x2 comparison of today vs graded 2 mm weighted."""
import sys
from PIL import Image, ImageDraw, ImageFont
S = '/tmp/claude-0/-home-user/800bed29-fd75-5057-8f10-928593efb56e/scratchpad/gui/q/'
m, title, left, right = sys.argv[1:5]
crop = tuple(map(int, sys.argv[5:9])) if len(sys.argv) > 8 else (150, 120, 900, 600)
x, y, w, h = crop
def font(size):
    for p in ('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', '/usr/share/fonts/dejavu/DejaVuSans.ttf'):
        try: return ImageFont.truetype(p, size)
        except OSError: pass
    return ImageFont.load_default()
f1, f2 = font(26), font(19)
pad, head, rowh = 12, 64, 34
sheet = Image.new('RGB', (2 * w + 3 * pad, head + 2 * (h + rowh) + 3 * pad), 'white')
d = ImageDraw.Draw(sheet)
d.text((pad, 14), title, fill='black', font=f1)
for c, (v, note) in enumerate((('today', left), ('graded2W', right))):
    for r, view in enumerate(('external', 'internal')):
        img = Image.open(S + '%s_%s_%s.png' % (m, v, view)).crop((x, y, x + w, y + h))
        ox, oy = pad + c * (w + pad), head + r * (h + rowh + pad)
        label = ('Today (auto cells)' if v == 'today' else 'Graded 2 mm, weighted') + (' - all layers' if view == 'external' else ' - up to half height')
        d.text((ox, oy + 6), label, fill='black', font=f2)
        sheet.paste(img, (ox, oy + rowh))
        if r == 0:
            tx, ty = ox + 8, oy + rowh + h - 32
            box = d.textbbox((tx, ty), note, font=f2)
            d.rectangle((box[0] - 6, box[1] - 5, box[2] + 6, box[3] + 5), fill='white')
            d.text((tx, ty), note, fill='black', font=f2)
sheet.save(S + 'sheet_%s.png' % m)
print(S + 'sheet_%s.png' % m, sheet.size)
