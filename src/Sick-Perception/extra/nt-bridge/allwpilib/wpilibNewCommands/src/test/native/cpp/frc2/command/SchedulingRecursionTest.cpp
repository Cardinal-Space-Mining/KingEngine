// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include "CommandTestBase.h"
#include "frc2/command/Command.h"
#include "frc2/command/CommandHelper.h"
#include "frc2/command/FunctionalCommand.h"
#include "frc2/command/RunCommand.h"

using namespace frc2;

class SchedulingRecursionTest
    : public CommandTestBaseWithParam<Command::InterruptionBehavior> {};

class SelfCancellingCommand
    : public CommandHelper<Command, SelfCancellingCommand> {
 public:
  SelfCancellingCommand(CommandScheduler* scheduler, int& counter,
                        Subsystem* requirement,
                        Command::InterruptionBehavior interruptionBehavior =
                            Command::InterruptionBehavior::kCancelSelf)
      : m_scheduler(scheduler),
        m_counter(counter),
        m_interrupt(interruptionBehavior) {
    AddRequirements(requirement);
  }

  void Initialize() override { m_scheduler->Cancel(this); }

  void End(bool interrupted) override { m_counter++; }

  InterruptionBehavior GetInterruptionBehavior() const override {
    return m_interrupt;
  }

 private:
  CommandScheduler* m_scheduler;
  int& m_counter;
  InterruptionBehavior m_interrupt;
};

/**
 * Checks <a
 * href="https://github.com/wpilibsuite/allwpilib/issues/4259">wpilibsuite/allwpilib#4259</a>.
 */
TEST_P(SchedulingRecursionTest, CancelFromInitialize) {
  CommandScheduler scheduler = GetScheduler();
  bool hasOtherRun = false;
  int counter = 0;
  TestSubsystem requirement;
  SelfCancellingCommand selfCancels{&scheduler, counter, &requirement,
                                    GetParam()};
  RunCommand other{[&hasOtherRun] { hasOtherRun = true; }, {&requirement}};

  scheduler.Schedule(&selfCancels);
  scheduler.Run();
  scheduler.Schedule(&other);

  EXPECT_FALSE(scheduler.IsScheduled(&selfCancels));
  EXPECT_TRUE(scheduler.IsScheduled(&other));
  EXPECT_EQ(1, counter);
  scheduler.Run();
  EXPECT_TRUE(hasOtherRun);
}

TEST_F(SchedulingRecursionTest, CancelFromInitializeAction) {
  CommandScheduler scheduler = GetScheduler();
  bool hasOtherRun = false;
  int counter = 0;
  TestSubsystem requirement;
  FunctionalCommand selfCancels{[] {},
                                [] {},
                                [&counter](bool) { counter++; },
                                [] { return false; },
                                {&requirement}};
  RunCommand other{[&hasOtherRun] { hasOtherRun = true; }, {&requirement}};
  scheduler.OnCommandInitialize([&scheduler, &selfCancels](const Command&) {
    scheduler.Cancel(&selfCancels);
  });
  scheduler.Schedule(&selfCancels);
  scheduler.Run();
  scheduler.Schedule(&other);

  EXPECT_FALSE(scheduler.IsScheduled(&selfCancels));
  EXPECT_TRUE(scheduler.IsScheduled(&other));
  EXPECT_EQ(1, counter);
  scheduler.Run();
  EXPECT_TRUE(hasOtherRun);
}

TEST_P(SchedulingRecursionTest,
       DefaultCommandGetsRescheduledAfterSelfCanceling) {
  CommandScheduler scheduler = GetScheduler();
  bool hasOtherRun = false;
  int counter = 0;
  TestSubsystem requirement;
  SelfCancellingCommand selfCancels{&scheduler, counter, &requirement,
                                    GetParam()};
  RunCommand other{[&hasOtherRun] { hasOtherRun = true; }, {&requirement}};
  scheduler.SetDefaultCommand(&requirement, std::move(other));

  scheduler.Schedule(&selfCancels);
  scheduler.Run();
  scheduler.Run();
  EXPECT_FALSE(scheduler.IsScheduled(&selfCancels));
  EXPECT_TRUE(scheduler.IsScheduled(scheduler.GetDefaultCommand(&requirement)));
  EXPECT_EQ(1, counter);
  scheduler.Run();
  EXPECT_TRUE(hasOtherRun);
}

class CancelEndCommand : public CommandHelper<Command, CancelEndCommand> {
 public:
  CancelEndCommand(CommandScheduler* scheduler, int& counter)
      : m_scheduler(scheduler), m_counter(counter) {}

  void End(bool interrupted) override {
    m_counter++;
    m_scheduler->Cancel(this);
  }

 private:
  CommandScheduler* m_scheduler;
  int& m_counter;
};

TEST_F(SchedulingRecursionTest, CancelFromEnd) {
  CommandScheduler scheduler = GetScheduler();
  int counter = 0;
  CancelEndCommand selfCancels{&scheduler, counter};

  scheduler.Schedule(&selfCancels);

  EXPECT_NO_THROW({ scheduler.Cancel(&selfCancels); });
  EXPECT_EQ(1, counter);
  EXPECT_FALSE(scheduler.IsScheduled(&selfCancels));
}

TEST_F(SchedulingRecursionTest, CancelFromInterruptAction) {
  CommandScheduler scheduler = GetScheduler();
  int counter = 0;
  FunctionalCommand selfCancels{[] {}, [] {}, [](bool) {},
                                [] { return false; }};
  scheduler.OnCommandInterrupt([&](const Command&) {
    counter++;
    scheduler.Cancel(&selfCancels);
  });
  scheduler.Schedule(&selfCancels);

  EXPECT_NO_THROW({ scheduler.Cancel(&selfCancels); });
  EXPECT_EQ(1, counter);
  EXPECT_FALSE(scheduler.IsScheduled(&selfCancels));
}

class EndCommand : public CommandHelper<Command, EndCommand> {
 public:
  explicit EndCommand(std::function<void(bool)> end) : m_end(end) {}
  void End(bool interrupted) override { m_end(interrupted); }
  bool IsFinished() override { return true; }

 private:
  std::function<void(bool)> m_end;
};

TEST_F(SchedulingRecursionTest, CancelFromEndLoop) {
  CommandScheduler scheduler = GetScheduler();
  int counter = 0;
  EndCommand dCancelsAll([&](bool) {
    counter++;
    scheduler.CancelAll();
  });
  EndCommand cCancelsD([&](bool) {
    counter++;
    scheduler.Cancel(&dCancelsAll);
  });
  EndCommand bCancelsC([&](bool) {
    counter++;
    scheduler.Cancel(&cCancelsD);
  });
  EndCommand aCancelsB([&](bool) {
    counter++;
    scheduler.Cancel(&bCancelsC);
  });
  scheduler.Schedule(&aCancelsB);
  scheduler.Schedule(&bCancelsC);
  scheduler.Schedule(&cCancelsD);
  scheduler.Schedule(&dCancelsAll);

  EXPECT_NO_THROW({ scheduler.Cancel(&aCancelsB); });
  EXPECT_EQ(4, counter);
  EXPECT_FALSE(scheduler.IsScheduled(&aCancelsB));
  EXPECT_FALSE(scheduler.IsScheduled(&bCancelsC));
  EXPECT_FALSE(scheduler.IsScheduled(&cCancelsD));
  EXPECT_FALSE(scheduler.IsScheduled(&dCancelsAll));
}

TEST_F(SchedulingRecursionTest, CancelFromEndLoopWhileInRunLoop) {
  CommandScheduler scheduler = GetScheduler();
  int counter = 0;
  EndCommand dCancelsAll([&](bool) {
    counter++;
    scheduler.CancelAll();
  });
  EndCommand cCancelsD([&](bool) {
    counter++;
    scheduler.Cancel(&dCancelsAll);
  });
  EndCommand bCancelsC([&](bool) {
    counter++;
    scheduler.Cancel(&cCancelsD);
  });
  EndCommand aCancelsB([&](bool) {
    counter++;
    scheduler.Cancel(&bCancelsC);
  });
  scheduler.Schedule(&aCancelsB);
  scheduler.Schedule(&bCancelsC);
  scheduler.Schedule(&cCancelsD);
  scheduler.Schedule(&dCancelsAll);

  EXPECT_NO_THROW({ scheduler.Run(); });
  EXPECT_EQ(4, counter);
  EXPECT_FALSE(scheduler.IsScheduled(&aCancelsB));
  EXPECT_FALSE(scheduler.IsScheduled(&bCancelsC));
  EXPECT_FALSE(scheduler.IsScheduled(&cCancelsD));
  EXPECT_FALSE(scheduler.IsScheduled(&dCancelsAll));
}

class MultiCancelCommand : public CommandHelper<Command, MultiCancelCommand> {
 public:
  MultiCancelCommand(CommandScheduler* scheduler, int& counter,
                     Command* command)
      : m_scheduler(scheduler), m_counter(counter), m_command(command) {}

  void End(bool interrupted) override {
    m_counter++;
    m_scheduler->Cancel(m_command);
    m_scheduler->Cancel(this);
  }

 private:
  CommandScheduler* m_scheduler;
  int& m_counter;
  Command* m_command;
};

TEST_F(SchedulingRecursionTest, MultiCancelFromEnd) {
  CommandScheduler scheduler = GetScheduler();
  int counter = 0;
  EndCommand bIncrementsCounter([&counter](bool) { counter++; });
  MultiCancelCommand aCancelsB{&scheduler, counter, &bIncrementsCounter};

  scheduler.Schedule(&aCancelsB);
  scheduler.Schedule(&bIncrementsCounter);

  EXPECT_NO_THROW({ scheduler.Cancel(&aCancelsB); });
  EXPECT_EQ(2, counter);
  EXPECT_FALSE(scheduler.IsScheduled(&aCancelsB));
  EXPECT_FALSE(scheduler.IsScheduled(&bIncrementsCounter));
}

TEST_P(SchedulingRecursionTest, ScheduleFromEndCancel) {
  CommandScheduler scheduler = GetScheduler();
  int counter = 0;
  TestSubsystem requirement;
  SelfCancellingCommand selfCancels{&scheduler, counter, &requirement,
                                    GetParam()};
  RunCommand other{[] {}, {&requirement}};

  scheduler.Schedule(&selfCancels);
  EXPECT_NO_THROW({ scheduler.Cancel(&selfCancels); });
  EXPECT_EQ(1, counter);
  EXPECT_FALSE(scheduler.IsScheduled(&selfCancels));
}

TEST_P(SchedulingRecursionTest, ScheduleFromEndInterrupt) {
  CommandScheduler scheduler = GetScheduler();
  int counter = 0;
  TestSubsystem requirement;
  SelfCancellingCommand selfCancels{&scheduler, counter, &requirement,
                                    GetParam()};
  RunCommand other{[] {}, {&requirement}};

  scheduler.Schedule(&selfCancels);
  EXPECT_NO_THROW({ scheduler.Schedule(&other); });
  EXPECT_EQ(1, counter);
  EXPECT_FALSE(scheduler.IsScheduled(&selfCancels));
  EXPECT_TRUE(scheduler.IsScheduled(&other));
}

TEST_F(SchedulingRecursionTest, ScheduleFromEndInterruptAction) {
  CommandScheduler scheduler = GetScheduler();
  int counter = 0;
  TestSubsystem requirement;
  RunCommand selfCancels{[] {}, {&requirement}};
  RunCommand other{[] {}, {&requirement}};
  scheduler.OnCommandInterrupt([&](const Command&) {
    counter++;
    scheduler.Schedule(&other);
  });
  scheduler.Schedule(&selfCancels);
  EXPECT_NO_THROW({ scheduler.Schedule(&other); });
  EXPECT_EQ(1, counter);
  EXPECT_FALSE(scheduler.IsScheduled(&selfCancels));
  EXPECT_TRUE(scheduler.IsScheduled(&other));
}

TEST_F(SchedulingRecursionTest, CancelDefaultCommandFromEnd) {
  CommandScheduler scheduler = GetScheduler();
  int counter = 0;
  TestSubsystem requirement;
  FunctionalCommand defaultCommand{[] {},
                                   [] {},
                                   [&counter](bool) { counter++; },
                                   [] { return false; },
                                   {&requirement}};
  RunCommand other{[] {}, {&requirement}};
  FunctionalCommand cancelDefaultCommand{[] {}, [] {},
                                         [&](bool) {
                                           counter++;
                                           scheduler.Schedule(&other);
                                         },
                                         [] { return false; }};

  EXPECT_NO_THROW({
    scheduler.Schedule(&cancelDefaultCommand);
    scheduler.SetDefaultCommand(&requirement, std::move(defaultCommand));

    scheduler.Run();
    scheduler.Cancel(&cancelDefaultCommand);
  });
  EXPECT_EQ(2, counter);
  EXPECT_FALSE(scheduler.IsScheduled(&defaultCommand));
  EXPECT_TRUE(scheduler.IsScheduled(&other));
}

INSTANTIATE_TEST_SUITE_P(
    SchedulingRecursionTests, SchedulingRecursionTest,
    testing::Values(Command::InterruptionBehavior::kCancelSelf,
                    Command::InterruptionBehavior::kCancelIncoming));
