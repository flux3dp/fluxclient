from g2fcpp import GcodeToFcodeCpp
from io import BytesIO, StringIO

ext_metadata = {}
fcode_output = BytesIO()


with open("/Users/simon/Downloads/yoyoyodragon.gcode", 'r') as f:
    m_GcodeToFcode = GcodeToFcodeCpp(ext_metadata=ext_metadata)
    m_GcodeToFcode.engine = 'cura'
    m_GcodeToFcode.process(f, fcode_output)
    path = m_GcodeToFcode.trim_ends(m_GcodeToFcode.fc)
    metadata = m_GcodeToFcode.md


with open("output.fc","wb") as f:
	f.write(fcode_output.getvalue())