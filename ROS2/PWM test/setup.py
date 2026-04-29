from setuptools import setup

package_name = 'pwm_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='austin',
    maintainer_email='austin@todo.todo',
    description='PWM test',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pwm_test = pwm_test.pwm_test:main',
            'pwm_hw_node = pwm_test.pwm_hw_node:main',
        ],
    },
)