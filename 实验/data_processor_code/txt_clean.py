def clean_file_after_text(file_path, search_text):
    """
    清除文本文件中包含指定文本的那一行及其后的所有内容
    
    参数:
        file_path (str): 文本文件路径
        search_text (str): 要搜索的文本内容
    """
    try:
        # 读取所有行
        with open(file_path, 'r', encoding='utf-8') as file:
            lines = file.readlines()
        
        # 查找包含指定文本的行号
        target_line = -1
        for i, line in enumerate(lines):
            if search_text in line:
                target_line = i
                break
        
        # 如果找不到指定文本
        if target_line == -1:
            print(f"未找到包含文本 '{search_text}' 的行")
            return
            
        # 只保留到目标行之前的内容
        kept_lines = lines[:target_line]
        
        # 写回文件
        with open(file_path, 'w', encoding='utf-8') as file:
            file.writelines(kept_lines)
            
        print(f"已成功清除第 {target_line + 1} 行(包含文本 '{search_text}')及其后的内容")
        
    except FileNotFoundError:
        print(f"找不到文件: {file_path}")
    except Exception as e:
        print(f"发生错误: {str(e)}")

# 使用示例
if __name__ == "__main__":
    file_path = "code_2_data_6.txt"  # 替换为你的文件路径
    search_text = "All parameters tested. Experiment completed."  # 替换为你要查找的文本内容
    clean_file_after_text(file_path, search_text)