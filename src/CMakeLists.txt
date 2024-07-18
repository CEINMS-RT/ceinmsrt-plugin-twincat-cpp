ADD_LIBRARY( PluginPositionAndTorqueTwinCat SHARED PluginPositionAndTorqueTwinCat.cpp
)


TARGET_LINK_LIBRARIES( PluginPositionAndTorqueTwinCat
	TcAdsDll
	FileIO
	XMLIKIDInterpreter
	XMLExecutionInterpreter
	${Boost_LIBRARIES}
	XercesC::XercesC
	AdsClient
	FileLogger
)

ADD_EXECUTABLE(TestPluginTwincat TestAds.cpp
)

TARGET_LINK_LIBRARIES(TestPluginTwincat
	PluginPositionAndTorqueTwinCat
)