import numpy as np
import geatpy as ea # 导入geatpy库
import time
"""
    the problem is a single objective optimization problem to minimize the objective function
    objective function
        f = alpha * d + beta * (t - tmin)^2 + gama * n
        d: distance between the box and target
        t: time step to accomplish the simulation
        tmin: minimal time step to accomplish the game
        n: number to hit the obstacle
    optimization parameters
        R1: threshold distance regard to task field converge line between obs and box [0,1]
        R2: threshold distance regard to social field between epuck [0,1]
        K1: parameter to calculate the converge line regard to the distance between obstacle and box [0,5]
        k2: attraction parameter to calculate the task field [0,5]
        k3: repellent parameter to calculate the task field [0,5]
        k4: repellent parameter to calculate the social field [0,1]
"""

class EA_algorithm():
    def __init__(self, POPULATION_SIZE, GENOTYPE_SIZE):
        ##########------------------------------------------variable setting--------------------------------------##########
        Dim = 6
        lb = [0, 0, 0, 0, 0, 0]
        ub = [1, 1, 5, 5, 5, 1]
        range = np.vstack([lb, ub])
        lbin = [0] * Dim
        ubin = [1] * Dim
        borders = np.vstack([lbin, ubin])
        varTypes = np.array([0] * Dim)
        ##########------------------------------------------encoding typing--------------------------------------##########
        self.Encoding = "BG"
        codes = [1] * Dim   #  gray code
        precisions = [1] * Dim
        scales = [0] * Dim
        self.FieldD = ea.crtfld(self.Encoding, varTypes, range, borders, precisions, codes, scales)
        self.NIND = POPULATION_SIZE
        self.MAXGEN = GENOTYPE_SIZE
        self.maxormins = np.array([1])
        self.selectStyle = "rws"    # Roulette Wheel Selection method
        self.recStyle = "xovdp"     # Two points cross
        self.mutStyle = "mutbin"    # binary mutation
        self.Lind = int(np.sum(self.FieldD[0, :])) # calculate the length of the chromosome
        self.pc = 0.7   # crossing Probability
        self.pm = 1/self.Lind
        self.obj_trace = np.zeros((self.MAXGEN, Dim))
        self.var_trace = np.zeros((self.MAXGEN, self.Lind))
        self.Chrom = ea.crtpc(self.Encoding, self.NIND, self.FieldD)
        self.generation = ea.bs2real(self.Chrom, self.FieldD)
        self.FitnV = []

    def reproduce_generation(self, ObjV, CV, gen):
        self.FitnV = ea.ranking(ObjV, CV, self.maxormins)
        best_ind = np.argmax(self.FitnV)
        self.obj_trace[gen, 0] = np.sum(ObjV) / ObjV.shape[0]
        self.obj_trace[gen, 1] = ObjV[best_ind]
        self.var_trace[gen, :] = self.Chrom[best_ind, :]
        SelCh = self.Chrom[ea.selecting(self.selectStyle, self.FitnV, self.NIND - 1), :]
        SelCh = ea.recombin(self.recStyle, SelCh, self.pc)
        SelCh = ea.mutate(self.mutStyle, self.Encoding, SelCh, self.pm)
        self.Chrom = np.vstack([self.Chrom[best_ind, :], SelCh])
        self.generation = ea.bs2real(self.Chrom, self.FieldD)
        return self.generation
