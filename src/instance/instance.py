import random
import os


def split_list(original_list, num_splits):
    """
    将list在随机位置划分为若干段
    Args:
        original_list: 原list
        num_splits: 需要划分为多少段

    Returns:
        新list组成的list[[],[],[]]

    """
    if num_splits < 2:
        return "Number of splits should be at least 2"
    split_indices = sorted(random.sample(range(1, len(original_list)), num_splits - 1))
    new_lists = []
    start_index = 0
    for index in split_indices:
        new_lists.append(original_list[start_index:index])
        start_index = index
    new_lists.append(original_list[start_index:])
    return new_lists


class Instance:
    def __init__(self, name):
        """
        初始化算例类
        """
        self.name = name
        self.mA = 3  # 大车出发点&大车&Block数量
        self.mK = 3  # 小车出发点数量
        self.nK = 5  # 小车数量
        self.mS = 3  # 拣选台数量
        self.m2 = 2  # 每个Block中暂存位数量
        self.m1 = 4 * self.m2  # 每个Block中存储位数量
        self.n = 10  # 拣选任务的数量
        self.task = self.task_gen()  # 所有拣选任务{task_id:存储位，拣选台，拣选时间}
        self.coor = self.coor_gen()  # 生成所有位置的坐标 {l:(x,y,z)}
        self.Aw = {i + 8 * self.n: i for i in range(self.mA)}  # 每个大车出发点绑定的大车{i:i}
        self.Kw = self.Kw_gen()  # 每个小车出发点绑定的小车 {i:[]}
        self.QA = 3  # 大车载量
        self.QK = 1  # 小车载量
        self.tau_i = self.tau_i_gen()  # 点i的取/放货时间 {i:tau_i}
        self.Tau = self.Tau_gen()  # 时间矩阵{(k,l):tau_kl}
        # 全点集
        self.P1 = list(range(self.n))
        self.D1 = list(range(self.n, 2 * self.n))
        self.P2 = list(range(2 * self.n, 3 * self.n))
        self.D2 = list(range(3 * self.n, 4 * self.n))
        self.P3 = list(range(4 * self.n, 5 * self.n))
        self.D3 = list(range(5 * self.n, 6 * self.n))
        self.P4 = list(range(6 * self.n, 7 * self.n))
        self.D4 = list(range(7 * self.n, 8 * self.n))
        self.WA = list(range(8 * self.n, 8 * self.n + self.mA))
        self.WK = list(range(8 * self.n + self.mA, 8 * self.n + self.mA + self.mK))
        # 全位置集
        self.M1 = {i: [i * self.m1 + j for j in range(self.m1)] for i in range(self.mA)}
        self.M2 = {i: [self.mA * self.m1 + i * self.m2 + j for j in range(self.m2)] for i in range(self.mA)}
        self.M3 = [self.mA * self.m1 + self.mA * self.m2 + j for j in range(self.mS)]
        self.M4 = [self.mA * self.m1 + self.mA * self.m2 + self.mS + j for j in range(self.mA)]
        self.M5 = [self.mA * self.m1 + self.mA * self.m2 + self.mS + self.mA + j for j in range(self.mK)]
        self.print_instance()

    def task_gen(self):
        """
        生成拣选任务
        Returns:
            {task_id:存储位，拣选台，拣选时间}

        """
        task = {}
        task_m1 = random.sample(range(self.m1 * self.mA), self.n)  # 任务所在存储位
        task_s = [random.randint(self.mA * self.m1 + self.mA * self.m2, self.mA * self.m1 + self.mA * self.m2 + self.mS)
                  for _ in range(self.n)]  # 任务所在拣选台
        task_t = [random.uniform(1, 5) for _ in range(self.n)]  # 任务所需的拣选时间
        for i in range(self.n):
            task[i] = [task_m1[i], task_s[i], task_t[i]]
        return task

    def coor_gen(self):
        """
        生成所有位置点的坐标
        Returns:
            {位置点:(x,y,z)}

        """
        coor = {}
        for i in range(self.m1):  # 存储位坐标
            y = i % self.m2 + 2
            z = i // self.m2 + 1
            for j in range(self.mA):
                x = j + 1
                coor[j * self.m1 + i] = (x, y, z)
        for i in range(self.m2):  # 暂存位坐标
            y = i + 2
            for j in range(self.mA):
                x = j + 1
                coor[self.mA * self.m1 + j * self.m2 + i] = (x, y, 0)
        for i in range(self.mS):
            coor[self.mA * self.m1 + self.mA * self.m2 + i] = (i + 1, 1, 0)  # 拣选台坐标
        for i in range(self.mA):
            coor[self.mA * self.m1 + self.mA * self.m2 + self.mS + i] = (i + 1, 2, 0)  # 大车起点坐标
        for i in range(self.mK):
            coor[self.mA * self.m1 + self.mA * self.m2 + self.mS + self.mA + i] = (i + 1, 1, 0)  # 小车起点坐标
        return coor

    def Kw_gen(self):
        """
        每个小车出发点绑定的小车
        Returns:
            {i:[]}

        """
        K = list(range(self.nK))
        Kw_list = split_list(K, self.mK)
        return {8 * self.n + self.mA + i: Kw_list[i] for i in range(self.mK)}

    def Tau_gen(self):
        """
        位置点之间移动时间矩阵
        Returns:
            {(i,j):tau_ij}

        """
        Tau = {}
        M = self.mA * self.m1 + self.mA * self.m2 + self.mS + self.mA + self.mK
        for i in range(M):
            for j in range(M):
                Tau[(i, j)] = abs(self.coor[i][0] - self.coor[j][0]) + abs(self.coor[i][1] - self.coor[j][1]) + abs(
                    self.coor[i][2] - self.coor[j][2])
        return Tau

    def tau_i_gen(self):
        """
        各点取放货时间
        Returns:
            {i:tau_i}

        """
        tau = {i: 3 for i in range(8 * self.n)}
        for i in range(8 * self.n, 8 * self.n + self.mA + self.mK):
            tau[i] = 0
        return tau

    def print_instance(self):
        """
        打印instance中全部信息
        Returns:

        """
        # 指定文件夹路径
        folder_path = 'instance'
        # 如果文件夹不存在，则创建
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)

        # 指定文件路径
        file_path = os.path.join(folder_path, f"{self.name}.txt")

        # 打开文件并写入内容
        with open(file_path, 'w') as file:
            file.write("INFO of {}:\n".format(self.name))
            file.write("Block、大车、大车出发点数量：{}\n".format(self.mA))
            file.write("小车出发点数量：{}\n".format(self.mK))
            file.write("小车数量：{}\n".format(self.nK))
            file.write("拣选台数量：{}\n".format(self.mS))
            file.write("每个Block中存储位数量：{}\n".format(self.m1))
            file.write("每个Block中暂存位数量：{}\n".format(self.m2))
            file.write("拣选任务数量：{}\n".format(self.n))
            file.write("所有拣选任务：存储位、拣选台、拣选时间\n")
            file.write(str(self.task) + '\n')
            file.write("所有位置坐标：\n")
            file.write(str(self.coor) + '\n')
            file.write("每个大车出发点绑定的大车：\n")
            file.write(str(self.Aw) + '\n')
            file.write("每个小车出发点绑定的大车：\n")
            file.write(str(self.Kw) + '\n')
            file.write("大车载量：{}\n".format(self.QA))
            file.write("小车载量：{}\n".format(self.QK))
            file.write("各点取、放货时间\n")
            file.write(str(self.tau_i) + '\n')
            file.write("各位置间移动时间矩阵\n")
            file.write(str(self.Tau) + '\n')
            file.write("附：\n")
            file.write("全点集：\n")
            file.write("P1 = {}\n".format(self.P1))
            file.write("D1 = {}\n".format(self.D1))
            file.write("P2 = {}\n".format(self.P2))
            file.write("D2 = {}\n".format(self.D2))
            file.write("P3 = {}\n".format(self.P3))
            file.write("D3 = {}\n".format(self.D3))
            file.write("P4 = {}\n".format(self.P4))
            file.write("D4 = {}\n".format(self.D4))
            file.write("WA = {}\n".format(self.WA))
            file.write("WK = {}\n".format(self.WK))
            file.write("全位置集：\n")
            file.write("M1 = {}\n".format(self.M1))
            file.write("M2 = {}\n".format(self.M2))
            file.write("M3 = {}\n".format(self.M3))
            file.write("M4 = {}\n".format(self.M4))
            file.write("M5 = {}\n".format(self.M5))

        print(f"信息已保存到文件: {file_path}")
