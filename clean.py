import re

# 1. 配置文件路径（请修改为你实际的文件名）
input_file = 'results0508_3_63.txt'      # 原始 rostopic 数据文件
output_file = 'sampled_63.txt'     # 提取后保存的新文件

def extract_and_sample():
    # 读取原始文件内容
    with open(input_file, 'r', encoding='utf-8') as f:
        content = f.read()

    # 按 '---' 将数据分割成一个个独立的块（每一帧位姿）
    blocks = content.split('---')
    
    # 定义正则表达式，只匹配 position 下的 x, y, z，忽略 orientation 里的 x,y,z
    # [eE\d\.\-\+]+ 用于精准匹配带有科学计数法、小数点及正负号的浮点数
    pattern = re.compile(r'position:\s*x:\s*([eE\d\.\-\+]+)\s*y:\s*([eE\d\.\-\+]+)\s*z:\s*([eE\d\.\-\+]+)')

    sampled_points = []

    # 2. 以 1/40 的采样率遍历数据块 (步长设为 40)
    for i in range(0, len(blocks), 40):
        block = blocks[i]
        match = pattern.search(block)
        
        if match:
            # 提取匹配到的 x, y, z
            x = float(match.group(1))
            y = float(match.group(2))
            z = float(match.group(3))
            
            # 将三个坐标按两位小数格式化，并用空格隔开
            # 格式例如: "-0.00 -0.00 -1.14"
            sampled_points.append(f"{x:.2f} {y:.2f} {z:.2f}")

    # 3. 将所有的点组合成要求的一行格式："x1 y1 z1 x2 y2 z2 x3 y3 z3 ..."
    result_str = " ".join(sampled_points)

    # 如果你想每个点占一行(x1 y1 z1 \n x2 y2 z2)，请将上面的 " ".join 改为 "\n".join

    # 写入新文件
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(result_str)

    print(f"处理完成！总数据帧数: {len(blocks)}，采样后得到 {len(sampled_points)} 个点。")
    print(f"结果已保存至: {output_file}")

if __name__ == '__main__':
    extract_and_sample()