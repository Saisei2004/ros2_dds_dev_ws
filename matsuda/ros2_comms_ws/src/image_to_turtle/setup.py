from setuptools import find_packages, setup

package_name = 'image_to_turtle'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python', 'numpy', 'requests'],
    zip_safe=True,
    maintainer='matsuda',
    maintainer_email='matsuda@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
            entry_points={
                'console_scripts': [
                    'image_downloader = image_to_turtle.image_downloader:main',
                    'line_detector = image_to_turtle.line_detector:main',
                    'turtle_drawer = image_to_turtle.turtle_drawer:main',
                    'image_to_turtle_main = image_to_turtle.image_to_turtle_main:main',
                    'simple_demo = image_to_turtle.simple_demo:main',
                    'simple_line_test = image_to_turtle.simple_line_test:main',
                ],
            },
)
