   M S B A M L   `   `   `  ����   DepthBasics-WPF5    /Microsoft.Samples.Kinect.DepthBasics.MainWindow   R NWindowsBase, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35W SPresentationCore, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35\ XPresentationFramework, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35D 9http://schemas.microsoft.com/winfx/2006/xaml/presentation    R NSystem.Xaml, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     -     � Title$  Depth Basics��$	��600��$	��700�� � 	Resources    MediumGreyBrush �� Color  KinectPurpleBrush  KinectBlueBrush �� 
TargetType �� Property �� Value  ScreenshotButton �� 
TargetType �� Orientation ��Name	 �� TextAlignment
 ��Name �� Triggers P� Property P� Value �� 
TargetName��%�  &	        &	      &	 *     '�� ?     &	 t     7  7  �� $ 	#ff6e6e6e���� $ 	#ff52318f���� $ 	#ff00BCF2���� " ��.���  �@J�SnapsToDevicePixels . �� " ��.���  � �� �� " ��.R�� .+��� $ 
Horizontal=�
������ .+��� $ ScreenshotText��#��z��$
	 Left=�$��Center=�8��  $��15(�� $��9,0,0,0q�.+��� $
 ScreenshotNormal��$!��Images\ScreenshotNormal.png��$
��None=�$��Center=��� $
 ScreenshotHover��$ ��Images\ScreenshotHover.png��$
��None=�$��Center=�$y�	Collapsed=� P�  �@J�IsMouseOver . . ��� $ ScreenshotNormal�� � y�$ 	Collapsed=��� $ ScreenshotHover�� � y�$ Visible=��� 8  $ ScreenshotText�� � ��.��� $��	10 0 10 0q� � RowDefinitions �� $
��Auto ��� $��* ��� $
��Auto ��� $
��Auto �.+��� $��0��$��Images\Logo.png��$
��Left=�$
��Fill=�$��32��$��81��$��0 10 0 5q��� $��0��$��0 0 -1 0q�$��Right=�$��Bottom=�#��[  $��Segoe UI)�$��18(�Depth Basics�� $��0��$��Images\Status.png��$
��None=�$��Center=�$��0 0 0 5q�(� $��1��$��Center=�.���� $��UniformToFill=����� *ImageSource+�� $��2��#��[ $��
Screenshot��$
��Auto��$��Right=�$��Center=�$��
10 10 0 10q��� -   $��	statusBar��$��3��$��Stretch=�$��Bottom=�
�������#��[  .@��� ���� *
StatusText+�� -    ��Name$ btnStartRecording��$��start��$
��Left=�$��	283,7,0,0q�$��2��$	��Top=�$��75���� -   $ btnStopRecording��$
��stop��$
��Left=�$��
283,37,0,0q�$��2��$	��Top=�$��75��$��2���� -   $ tbImagePath��$
��Left=�$��	377,7,0,0q�$��2��$
��Wrap=�$��D:/ZY/Pmomo/Data/Cloth2/��$	��Top=�$��20��$	��153���� -   $ tbBackgroundPath��$
��Left=�$��0,32,0,0q�$��2��$
��Wrap=�$	��Top=�$	��255��$,��&D:/ZY/Pmomo/Data/Cloth2/background.png���� -   $ btnSaveBackground��$��save background��$
��Left=�$��0,7,0,0q�$��2��$	��Top=�$	��110���� -   $ btnLoadBackground��$��load background��$
��Left=�$��	124,7,0,0q�$��2��$	��Top=�$	��113��