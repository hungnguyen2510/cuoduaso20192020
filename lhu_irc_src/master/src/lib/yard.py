#!/usr/bin/env python
# -*- coding: utf-8 -*-


class Yard:

    def init_san(self):
        self.ds_san = iter(["Left", "Right", "Test"])
        self.san = next(self.ds_san)
        self.ds_ham = iter(["Exam"])

    def init_ham(self):
        if self.san == 'Left' or self.san == 'Right':
            self.ds_ham = iter(["Exam"])
        if self.san == 'Test':
            self.ds_ham = iter(["Climb Left", "Climb Mid", "Climb Right", "Switch To Left", "Switch To Right"])
        self.ham = next(self.ds_ham)

    def __init__(self):
        self.init_san()
        self.init_ham()

    def chuyen_san(self):
        try:
            self.san = next(self.ds_san)
        except StopIteration:
            self.init_san()
        self.init_ham()

    def chuyen_ham(self):
        try:
            self.ham = next(self.ds_ham)
        except StopIteration:
            self.init_ham()
