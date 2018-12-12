using_task_library 'joint_tools'

describe OroGen.joint_tools.EffortPWMPositionerTask do
    attr_reader :now, :target, :position
    before do
        syskit_stub_conf(OroGen.joint_tools.EffortPWMPositionerTask, 'default',
            data: {
                'K' => [0.01, 0.02],
                'efforts' => [10, 20],
                'cycle_duration' => Time.at(1)
            }
        )

        @now = Time.now
        @target = Types.base.samples.Joints.from_positions(10, 10)
        @target.time = @now
        @position = Types.base.samples.Joints.from_positions(20, 0)
        @position.time = @now
    end

    describe 'size validation' do
        it "fails to configure if the size of the K and efforts property don't match" do
            syskit_stub_conf(OroGen.joint_tools.EffortPWMPositionerTask, 'default',
                data: {
                    'K' => [1]
                }
            )
            deployed = syskit_deploy(subject_syskit_model)
            expect_execution.scheduler(true).to do
                fail_to_start deployed, reason: Orocos::StateTransitionFailed
            end
        end

        describe "at runtime" do
            attr_reader :deployed
            before do
                @deployed = syskit_deploy_configure_and_start(subject_syskit_model)
                syskit_write @deployed.joints_target_port,
                    Types.base.commands.Joints.from_positions(10, 20)
                syskit_write @deployed.joints_position_port,
                    Types.base.commands.Joints.from_positions(10, 20)
            end

            it "reports UNEXPECTED_TARGET_SIZE if the joints_target sample has a size "\
                "that does not match the properties" do
                expect_execution do
                    syskit_write deployed.joints_target_port,
                        Types.base.commands.Joints.from_positions(10)
                end.to do
                    emit deployed.unexpected_target_size_event
                end
            end
            it "reports UNEXPECTED_POSITION_SIZE if the joints_position sample has a size "\
                "that does not match the properties" do
                expect_execution do
                    syskit_write deployed.joints_position_port,
                        Types.base.commands.Joints.from_positions(10)
                end.to do
                    emit deployed.unexpected_position_size_event
                end
            end
            it "reports NO_POSITION_IN_TARGET if the joints_target sample "\
                "does not have a valid position" do
                expect_execution do
                    syskit_write deployed.joints_target_port,
                        Types.base.commands.Joints.from_efforts(10, 20)
                end.to do
                    emit deployed.no_position_in_target_event
                end
            end
            it "reports NO_POSITION_IN_POSITION if the joints_position sample "\
                "does not have a valid position" do
                expect_execution do
                    syskit_write deployed.joints_position_port,
                        Types.base.commands.Joints.from_efforts(10, 20)
                end.to do
                    emit deployed.no_position_in_position_event
                end
            end
        end
    end

    describe "runtime behavior" do
        attr_reader :deployed
        before do
            @deployed = syskit_deploy_configure_and_start(subject_syskit_model)
            syskit_write deployed.joints_target_port, target
            @cmd = write_position_barrier(position)
        end

        it "sends a valid effort" do
            assert_equal [-10, 20], @cmd.elements.map(&:effort)
        end

        it "ignores changes in command within the cycle" do
            target.elements[0].position = 100
            syskit_write deployed.joints_target_port, target
            sample = write_position_barrier(position, time: now + 0.01)
            assert_equal [-10, 20], sample.elements.map(&:effort)
        end

        it "ignores changes in state within the cycle" do
            position.elements[0].position = 100
            sample = write_position_barrier(position, time: now + 0.01)
            assert_equal [-10, 20], sample.elements.map(&:effort)
        end

        it "switches to OFF at the expected deadline" do
            # current = [10, 10], target = [20, 0], K=[0.01, 0.02]
            # duty = [0.1, 0.2]
            sample = write_position_barrier(position, time: now + 0.1001)
            assert_equal [0, 20], sample.elements.map(&:effort)
            sample = write_position_barrier(position, time: now + 0.2001)
            assert_equal [0, 0], sample.elements.map(&:effort)
        end

        it "ues the last target and state to compute the next cycle" do
            position.elements[0].position = 0
            target.elements[1].position = 20
            # current=[0, 10] target=[20, 20]
            syskit_write deployed.joints_target_port, target
            # current = [0, 10], target = [20, 20], K=[0.01, 0.02]
            sample = write_position_barrier(position, time: now + 1.001)
            assert_equal [10, 20], sample.elements.map(&:effort)
        end
    end

    def write_position_barrier(position, time: nil)
        position = position.dup
        position.time = time if time
        expect_execution do
            syskit_write deployed.joints_position_port, position
        end.to do
            have_one_new_sample deployed.joints_cmd_port
        end
    end
end