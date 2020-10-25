import gym

env = gym.make('carla-v0')

# obs = env.reset()

episode_count = 10
reward = 0
done = False


for i in range(episode_count):
    ob = env.reset()
    while True:
        action = env.action_space.sample()
        ob, reward, done, _ = env.step(action)
        print('reward : ', reward)
        # env.render()
        if done:
            break
        # Note there's no env.render() here. But the environment still can open window and
        # render if asked by env.monitor: it calls env.render('rgb_array') to record video.
        # Video is not recorded every episode, see capped_cubic_video_schedule for details.

# Close the env and write monitor result info to disk
env.close()
